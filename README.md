<!--
SPDX-FileCopyrightText: 2022 Daniel Griswold

SPDX-License-Identifier: MIT
-->

# Air Quality Monitor


## Overview

This repository contains code and details that collect local environmental data and publish that data to MQTT.  Data points include temperature, humidity, CO2, VOC, and particulate matter.

MQTT is a standard, lightweight, messaging protocol for IoT devices and provides a nice medium for integrations into other systems, such as Home Assistant or InfluxDB.

Because the SGP30 requires baseline values, the current baseline is regularly written to non-volatile memory and read at startup.


## Hardware

- Feather development board https://www.adafruit.com/product/4769
  - CircuitPython compatible
  - Standard wifi library (not airlift)
  > Note: This was built on an Unexpected Maker FeatherS2 using CircuitPython 7.1.0, but should be compatible with other wifi-enabled boards and later CircuitPython versions.
- SGP30 MOX Gas Sensor https://www.adafruit.com/product/3709
- VCNL4040 Proximity/Lux Sensor https://www.adafruit.com/product/4161
- BME680 Pressure/Humidity/Temperature/Gas Sensor https://www.adafruit.com/product/3660
- 2.5 Particulate Matter Sensor over I2C https://www.adafruit.com/product/4632

Optional:
- 4 STEMMA QT cables https://www.adafruit.com/product/4399


## 3D Printed Stand

This is an open-bench style stand for the MCU and sensors. A full enclosure would require multiple chambers to isolate the temperature sensor from the heat generated by the MCU and fan in the PMS.  Additional engineering consideration would be needed to avoid impeding the light sensor.

 https://www.tinkercad.com/things/aU5Z6n3TVC7 (Licensed under CC BY-SA 3.0)

 <img src="https://user-images.githubusercontent.com/15717486/151087140-696f21f4-c8c2-4ca7-af3f-eb3fec597e32.jpg" width=504 height=672>


 ## Install

 1. Copy the repository contents to the CIRCUITPY drive.
 2. Use circup to install the required libraries. [Circup Overview](https://learn.adafruit.com/keep-your-circuitpython-libraries-on-devices-up-to-date-with-circup)

         circup install -r requirements.txt

3. Copy secrets.example to secrets.py and edit with appropriate values.

Other settings in code.py
- UPDATE_INTERVAL = 60 # Frequency to publish sensor data to MQTT in seconds.
- BASELINE_UPDATE_INTERVAL = 1800 # Frequency to write SGP Baseline to nvm in seconds.


## Sample output to MQTT

/environment:
    {
        "pressure": 995.703,
        "temperature": 23.8124,
        "light": 197.5,
        "TVOC": 33,
        "humidity": 26.1576,
        "eCO2": 420,
        "VOC": 13.031
    }

/air-quality:
    {
        "pm10 env": 0,
        "pm100 env": 0,
        "pm100 standard": 0,
        "particles 03um": 45,
        "pm25 standard": 0,
        "particles 10um": 0,
        "pm10 standard": 0,
        "pm25 env": 0,
        "particles 05um": 15,
        "particles 25um": 0,
        "particles 100um": 0,
        "particles 50um": 0
    }

/system:
    {
        "time": 3237.76,
        "ip_address": 192.168.x.x,
        "board_id": "unexpectedmaker_feathers2",
        "reset_reason": "POWER_ON"
    }
