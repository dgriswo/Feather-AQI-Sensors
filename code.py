import board
import time
import wifi
import ssl
import socketpool
import json
from math import exp, log
import digitalio
import microcontroller
import watchdog

microcontroller.watchdog.timeout = 30
microcontroller.watchdog.mode = watchdog.WatchDogMode.RESET
microcontroller.watchdog.feed()

import adafruit_bme680
import adafruit_sgp30
from adafruit_pm25.i2c import PM25_I2C
import adafruit_vcnl4040
import adafruit_minimqtt.adafruit_minimqtt as MQTT
import adafruit_dotstar
import foamyguy_nvm_helper as nvm_helper

from secrets import secrets

RESET_PIN = None
UPDATE_INTERVAL = 60
BASELINE_UPDATE_INTERVAL = 1800
MQTT_ENVIRONMENT = secrets["mqtt_topic"] + "/environment"
MQTT_AIR_QUALITY = secrets["mqtt_topic"] + "/air-quality"
MQTT_SYSTEM = secrets["mqtt_topic"] + "/system"


def sgp30_baseline_to_nvm(co2eq_base, tvoc_base):
    nvm_helper.save_data(
        {"co2eq_base": co2eq_base, "tvoc_base": tvoc_base}, test_run=False
    )


def sgp30_nvm_to_baseline():
    _data = nvm_helper.read_data()
    try:
        sgp30.set_iaq_baseline(_data["co2eq_base"], _data["tvoc_base"])
    except (RuntimeError, ValueError, EOFError) as error:
        print("Could not get baseline from nvm, setting defaults")
        sgp30.set_iaq_baseline(0x8973, 0x8AAE)


def sgp30_get_data(temperature, humidity):
    sgp30.set_iaq_humidity(computeAbsoluteHumidity(temperature, humidity))
    sgp30.iaq_measure()
    return sgp30.TVOC, sgp30.eCO2


def get_sensor_data():
    _temperature = bme680.temperature
    _humidity = bme680.humidity
    _pressure = bme680.pressure
    _r_gas = bme680.gas

    _TVOC, _eCO2 = sgp30_get_data(_temperature, _humidity)

    _data = {}
    _data["temperature"] = _temperature
    _data["humidity"] = _humidity
    _data["pressure"] = _pressure
    _data["light"] = vcnl4040.lux
    _data["VOC"] = computeIndoorAirQuality(_r_gas, _humidity)
    _data["TVOC"] = _TVOC
    _data["eCO2"] = _eCO2
    return _data


def get_system_data():
    _data = {}
    _data["reset_reason"] = str(microcontroller.cpu.reset_reason)[28:]
    _data["time"] = time.monotonic()
    _data["ip_address"] = wifi.radio.ipv4_address
    _data["board_id"] = board.board_id
    return _data


def computeAbsoluteHumidity(temperature, humidity):
    _absTemperature = temperature + 273.15
    _absHumidity = 6.112
    _absHumidity *= exp((17.67 * temperature) / (243.5 + temperature))
    _absHumidity *= humidity
    _absHumidity *= 2.1674
    _absHumidity /= _absTemperature
    return round(_absHumidity, 2)


def computeIndoorAirQuality(resistance, humidity):
    return log(resistance) + 0.04 * humidity


bme680 = adafruit_bme680.Adafruit_BME680_I2C(board.I2C())
sgp30 = adafruit_sgp30.Adafruit_SGP30(board.I2C())
pm25 = PM25_I2C(board.I2C(), RESET_PIN)
vcnl4040 = adafruit_vcnl4040.VCNL4040(board.I2C())
pixel = adafruit_dotstar.DotStar(
    board.APA102_SCK, board.APA102_MOSI, 1, brightness=0.1, auto_write=True
)
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

pixel[0] = (0, 255, 0)

try:
    microcontroller.watchdog.feed()
    print("Connecting to %s..." % secrets["ssid"])
    wifi.radio.connect(secrets["ssid"], secrets["password"])
    print("Connected to %s!" % secrets["ssid"])
    pool = socketpool.SocketPool(wifi.radio)
except Exception as e:
    print("Could not initialize network. {}".format(e))
    raise

try:
    microcontroller.watchdog.feed()
    mqtt_client = MQTT.MQTT(
        broker=secrets["mqtt_broker"],
        port=secrets["mqtt_port"],
        username=secrets["mqtt_username"],
        password=secrets["mqtt_password"],
        socket_pool=pool,
        ssl_context=ssl.create_default_context(),
    )
    mqtt_client.connect()
except MQTT.MMQTTException as e:
    print("Could not connect to mqtt broker. {}".format(e))
    raise

sgp30_nvm_to_baseline()
sgp30.iaq_init()

last_update = 0
last_iaq = 0
baseline_last_update = time.monotonic()

while True:
    _now = time.monotonic()
    microcontroller.watchdog.feed()

    if last_iaq + 1 < _now:
        # After the “sgp30_iaq_init” command, a
        # “sgp30_measure_iaq” command has to be sent in regular intervals
        # of 1s to ensure proper operation of the dynamic baseline
        # compensation algorithm.
        last_iaq = _now
        sgp30_get_data(bme680.temperature, bme680.humidity)

    if last_update + UPDATE_INTERVAL < _now:
        led.value = True
        last_update = _now

        print("Publishing AQI data")
        try:
            mqtt_client.publish(MQTT_AIR_QUALITY, json.dumps(pm25.read()), retain=True)
        except MQTT.MMQTTException as e:
            print("Could not publish to mqtt broker. {}".format(e))
        except RuntimeError:
            print("Could not read from PM25 sensor. {}".format(e))

        print("Publishing environmental data")
        try:
            mqtt_client.publish(
                MQTT_ENVIRONMENT, json.dumps(get_sensor_data()), retain=True
            )
        except MQTT.MMQTTException as e:
            print("Could not publish to mqtt broker. {}".format(e))

        print("Publishing system data")
        try:
            mqtt_client.publish(MQTT_SYSTEM, json.dumps(get_system_data()), retain=True)
        except MQTT.MMQTTException as e:
            print("Could not publish to mqtt broker. {}".format(e))

        led.value = False

    if baseline_last_update + BASELINE_UPDATE_INTERVAL < _now:
        baseline_last_update = _now
        sgp30_baseline_to_nvm(sgp30.baseline_eCO2, sgp30.baseline_TVOC)
