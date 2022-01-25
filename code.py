import board
import time
import wifi
import ssl
import socketpool
import json
from math import exp,log
import analogio
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

from secrets import secrets

RESET_PIN = None
UPDATE_INTERVAL = 60
BASELINE_UPDATE_INTERVAL = 10


def message(client, topic, message):
    if topic == secrets["mqtt_baseline_topic"]:
        _baseline = json.loads(message)
        try:
            print("Setting baselines from {}.".format(topic))
            sgp30.set_iaq_baseline(_baseline["co2eq_base"], _baseline["tvoc_base"])
        except (RuntimeError, ValueError) as error:
            print("Could not get baseline from mqtt, setting defaults")
            sgp30.set_iaq_baseline(0x8973, 0x8AAE)
        finally:
            mqtt_client.unsubscribe(topic)


def connected(client, userdata, flags, rc):
    client.subscribe(secrets["mqtt_baseline_topic"])


def sgp30_publish_baseline(co2eq_base, tvoc_base):
    _message = {
        "co2eq_base": co2eq_base,
        "tvoc_base": tvoc_base,
    }
    print("Publishing baselines")
    mqtt_client.publish(
        secrets["mqtt_baseline_topic"], json.dumps(_message), retain=True
    )


def sgp30_get_data():
    sgp30.iaq_measure()
    return sgp30.TVOC, sgp30.eCO2


def read_sensors():
    print("Getting temperature and humidity")
    _temperature = bme680.temperature
    _humidity = bme680.humidity
    _r_gas = bme680.gas

    print("Setting absolute humidity in SGP30")
    sgp30.set_iaq_humidity(computeAbsoluteHumidity(_temperature, _humidity))
    
    print("Reading sensors")
    _TVOC, _eCO2 = sgp30_get_data()
    try:
        aqi = pm25.read()
        _data = {
            "environmental": {
                "temperature": _temperature,
                "humidity": _humidity,
                "pressure": bme680.pressure,
                "light": vcnl4040.lux,
            },
            "gas": {
                "VOC": computeIndoorAirQuality(_r_gas, _humidity),
                "TVOC_Baseline": sgp30.baseline_TVOC,
                "TVOC": _TVOC,
                "eCO2_Baseline": sgp30.baseline_eCO2,
                "eCO2": _eCO2,
            },
            "aqi": aqi,
        }
    except RuntimeError:
        _data = {}
        pass

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
    mqtt_client.on_message = message
    mqtt_client.on_connect = connected
    mqtt_client.connect()
except MQTT.MMQTTException as e:
    print("Could not connect to mqtt broker. {}".format(e))

print("Initializing SGP30")
sgp30.iaq_init()
mqtt_client.loop()
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
        sgp30_get_data()
        print("eCO2 = %d ppm \t TVOC = %d ppb" % (sgp30.eCO2, sgp30.TVOC))

    if last_update + UPDATE_INTERVAL < _now:
        led.value = True
        last_update = _now
        output = read_sensors()
        print(output)
        print("Publishing sensor data")
        try:
            mqtt_client.publish(secrets["mqtt_topic"], json.dumps(output), retain=True)
        except Exception as e:
            continue

        led.value = False

    if baseline_last_update + BASELINE_UPDATE_INTERVAL < _now:
        baseline_last_update = _now
        pixel[0] = (255, 0, 0)
        print(
            "**** Baseline values: eCO2 = 0x%x, TVOC = 0x%x"
            % (sgp30.baseline_eCO2, sgp30.baseline_TVOC)
        )
        sgp30_publish_baseline(
            sgp30.baseline_eCO2, sgp30.baseline_TVOC
        )
        pixel[0] = (0, 255, 0)
