#!/usr/bin/env python

# Import standard python modules
import time
import csv
import os
import sys

# import Adafruit Blinka
import board
import busio

# import sensor libraries
from digitalio import DigitalInOut, Direction, Pull
from adafruit_pm25.i2c import PM25_I2C
from adafruit_ms8607 import MS8607
import adafruit_tsl2591
import adafruit_sgp30

# Create busio I2C
i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)

# create SGP30 object
sgp30 = adafruit_sgp30.Adafruit_SGP30(i2c)

print("SGP30 serial #", [hex(i) for i in sgp30.serial])
sgp30.iaq_init()
sgp30.set_iaq_baseline(0x8973, 0x8AAE)

voca = int(sgp30.eCO2)
vocb = int(sgp30.TVOC)

# create AQI object (PMSA0031 PM25)
reset_pin = None
pm25 = PM25_I2C(i2c, reset_pin)

# create Light sensor (TSL2591)
lightSensor = adafruit_tsl2591.TSL2591(i2c)

# create PHT object (MS8607)
phtSensor = MS8607(i2c)


# begin loop, start AQI read
time.sleep(10)  # wait for sensor to warm up
i = 0
while True:
    try:
        print("Iterations: ", i)
        localtime = time.asctime(time.localtime(time.time()))
        print("Time:", localtime)
        aqdata = pm25.read()
        print("PM2.5", aqdata["pm25 env"])
        AQI = float(aqdata["pm25 env"])

        # read SGP30 
        print("eCO2 = %d ppm \t TVOC = %d ppb" % (sgp30.eCO2, sgp30.TVOC))
        print("Baselines: eCO2 = 0x%x, TVOC = 0x%x" % (sgp30.baseline_eCO2, sgp30.baseline_TVOC)


        # read Light
        print('Light: {0}lux'.format(lightSensor.lux))
        print('Visible: {0}'.format(lightSensor.visible))
        print('Infrared: {0}'.format(lightSensor.infrared))
        light = float(round(lightSensor.lux, 2))

        # read PHT
        print("Pressure: %.2f hPa" % phtSensor.pressure)
        print("Temperature: %.2f C" % phtSensor.temperature)
        print("Humidity: %.2f %% rH" % phtSensor.relative_humidity)
        pressure = float(phtSensor.pressure)
        temperature = float(phtSensor.temperature)
        humidity = float(round(phtSensor.relative_humidity, 2))

        # write to CSV
        sensorVals = [i, localtime, AQI, pressure, temperature, humidity]
        sensorVals_floats = zip(sensorVals)

        path = '/home/pi/SensorValues.csv'
        os.path.join(path, "home", "SensorValues.csv", "/pi")
        fd = os.open("SensorValues.csv", os.O_APPEND|os.O_CREAT)



        with open('SensorValues.csv', 'a', newline='') as csvfile:
            fieldnames = ['Iterations', 'Time', 'AQI', 'VOC_eCO2', 'VOC_TVOC', 'Pressure', 'Temp', 'Humidity']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames, delimiter=',')
            writer.writerow({'Iterations': i, 'Time': localtime, 'AQI': AQI, 'VOC_eCO2': voca, 'VOC_TVOC': vocb,
                             'Pressure': pressure, 'Temp': temperature, 'Humidity': humidity})

        csvfile.close()

        time.sleep(60)
        i += 1
    except (RuntimeError, TypeError, NameError, ValueError):
        continue
