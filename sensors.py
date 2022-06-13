MQTT_TEMPERATURE_CONFIG = "homeassistant/sensor/office/temperature/config"
temperature = {
    "entity_id": "office_temperature",
    "device_class": "temperature",
    "name": "Office Temperature",
    "state_topic": "homeassistant/sensor/office/temperature/state",
    "unit_of_measurement": "°C",
}

MQTT_HUMIDITY_CONFIG = "homeassistant/sensor/office/humidity/config"
humidity = {
    "entity_id": "office_humidity",
    "device_class": "humidity",
    "name": "Office Humidity",
    "state_topic": "homeassistant/sensor/office/humidity/state",
    "unit_of_measurement": "%",
}

MQTT_PRESSURE_CONFIG = "homeassistant/sensor/office/pressure/config"
pressure = {
    "entity_id": "office_pressure",
    "device_class": "pressure",
    "name": "Office Pressure",
    "state_topic": "homeassistant/sensor/office/pressure/state",
    "unit_of_measurement": "hPA",
}

MQTT_LIGHT_CONFIG = "homeassistant/sensor/office/light/config"
light = {
    "entity_id": "office_light",
    "device_class": "illuminance",
    "name": "Office Light Level",
    "state_topic": "homeassistant/sensor/office/light/state",
    "unit_of_measurement": "lx",
}

MQTT_CO2_CONFIG = "homeassistant/sensor/office/co2/config"
co2 = {
    "entity_id": "office_co2",
    "device_class": "carbon_dioxide",
    "name": "Office CO2",
    "state_topic": "homeassistant/sensor/office/co2/state",
    "unit_of_measurement": "ppm",
}

MQTT_VOC_CONFIG = "homeassistant/sensor/office/voc/config"
voc = {
    "entity_id": "office_voc",
    "device_class": "volatile_organic_compounds",
    "name": "Office VOC",
    "state_topic": "homeassistant/sensor/office/voc/state",
    "unit_of_measurement": "ppb",
}

MQTT_AQI_CONFIG = "homeassistant/sensor/office/aqi/config"
aqi = {
    "entity_id": "office_aqi",
    "device_class": "aqi",
    "name": "Office AQI",
    "state_topic": "homeassistant/sensor/office/aqi/state",
}

MQTT_PM1_CONFIG = "homeassistant/sensor/office/pm1/config"
pm1 = {
    "entity_id": "office_pm1",
    "device_class": "pm1",
    "name": "Office 1 um",
    "state_topic": "homeassistant/sensor/office/pm1/state",
    "unit_of_measurement": "µg/m³",
}

MQTT_PM25_CONFIG = "homeassistant/sensor/office/pm25/config"
pm25 = {
    "entity_id": "office_pm25",
    "device_class": "pm25",
    "name": "Office 2.5 um",
    "state_topic": "homeassistant/sensor/office/pm25/state",
    "unit_of_measurement": "µg/m³",
}

MQTT_PM10_CONFIG = "homeassistant/sensor/office/pm10/config"
pm10 = {
    "entity_id": "office_pm10",
    "device_class": "pm10",
    "name": "Office 10 um",
    "state_topic": "homeassistant/sensor/office/pm10/state",
    "unit_of_measurement": "µg/m³",
}
