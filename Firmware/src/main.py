import os
import time
import board
import busio

import paho.mqtt.client as mqtt # paho-mqtt from https://github.com/eclipse/paho.mqtt.python.git
import adafruit_bmp280 # adafruit-circuitpython-bmp280 from https://github.com/adafruit/Adafruit_CircuitPython_BMP280.git
import adafruit_hcsr04 # adafruit-circuitpython-hcsr04 from https://github.com/adafruit/Adafruit_CircuitPython_HCSR04.git
import mpu6050 # mpu6050-raspberrypi from https://github.com/m-rtijn/mpu6050.git
import py_qmc5883l # py_qmc5883l from https://github.com/RigacciOrg/py-qmc5883l.git

import adafruit_ads1x15.ads1015 as ADS # adafruit-circuitpython-ads1x15 from https://github.com/adafruit/Adafruit_CircuitPython_ADS1x15.git
from adafruit_ads1x15.analog_in import AnalogIn

from ublox_gps import UbloxGps # sparkfun-ublox-gps from https://github.com/sparkfun/Qwiic_Ublox_Gps_Py.git
import serial

mqtt_user = os.getenv('MQTT_USER')
mqtt_pass = os.getenv('MQTT_PASS')

mqtt_broker = "192.168.0.132"
mqtt_port = 1883

topic = {
    "lon" : "rover/position/lon",
    "lat" : "rover/position/lat",
    "heading" : "rover/position/heading",
    "timestamp" : "rover/system/timestamp",
    "acc_x" : "rover/system/acc_x",
    "acc_y" : "rover/system/acc_y",
    "acc_z" : "rover/system/acc_z",
    "obstacle" : "rover/system/obstacle",
    "brightness" : "rover/environment/brightness",
    "temperature" : "rover/environment/temperature",
    "pressure" : "rover/environment/pressure"
}


def mqtt_connect():
    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqttc.on_message = lambda mqttc, obj, msg: (print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload)))
    mqttc.on_connect = lambda mqttc, obj, flags, reason_code, properties: (print("reason_code: " + str(reason_code)))
    mqttc.on_publish = lambda mqttc, obj, mid, reason_code, properties: (print("mid: " + str(mid)))
    mqttc.on_subscribe = lambda mqttc, obj, mid, reason_code_list, properties: (print("Subscribed: " + str(mid) + " " + str(reason_code_list)))

    # mqttc.on_log = lambda mqttc, obj, level, string: (print(string))

    mqttc.username_pw_set(username=mqtt_user,password=mqtt_pass)

    mqttc.connect(mqtt_broker, 1883, 60)

    mqttc.loop_start()

    return mqttc


if __name__ == '__main__':
    mqttc = mqtt_connect()

    i2c = busio.I2C(board.SCL, board.SDA)

    sonar = adafruit_hcsr04.HCSR04(trigger_pin=board.D23, echo_pin=board.D24)

    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)
    bmp280.sea_level_pressure = 1011 # pressure at sea level at current location

    mpu = mpu6050.mpu6050(0x68)

    ads = ADS.ADS1015(i2c)
    ldr = AnalogIn(ads, ADS.P0) # Create single-ended input on channel 0

    compass = py_qmc5883l.QMC5883L()
    compass.declination = 5.3 # magnetic declination in Vienna as of May 2024 (https://www.zamg.ac.at/cms/de/geophysik/produkte-und-services-1/online-deklinationsrechner)
    compass.calibration = [[1.011353814274526, 0.0003112697421351132, 556.2405586464367], [0.00031126974213510974, 1.0000085335949689, 3395.165166454792], [0.0, 0.0, 1.0]]

    try:
        ardusimple_port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
        gps = UbloxGps(ardusimple_port)
    except serial.SerialException as e:
        print("Ardusimple not connected!")
        gps = None

    while(True):
        ldr_resistance = (3.3-ldr.voltage)/ldr.voltage * 2200
        brightness = 12518931 * pow(ldr_resistance,-1.405) # Regression approximation from https://www.allaboutcircuits.com/projects/design-a-luxmeter-using-a-light-dependent-resistor/
    
        acc = mpu.get_accel_data()

        if(gps!=None):
            try:
                coords = gps.geo_coords()
                gps_time = gps.date_time()

                infot = mqttc.publish(topic["timestamp"], "UTC Time {}:{}:{}".format(gps_time.hour, gps_time.min, gps_time.sec), qos=2)
                infot.wait_for_publish()
                infot = mqttc.publish(topic["lon"], coords.lon, qos=2)
                infot.wait_for_publish()
                infot = mqttc.publish(topic["lat"], coords.lat, qos=2)
                infot.wait_for_publish()
            except (ValueError, IOError) as err:
                print(err)

        infot = mqttc.publish(topic["heading"], compass.get_bearing(), qos=2)
        infot.wait_for_publish()
        infot = mqttc.publish(topic["acc_x"], acc["x"], qos=2)
        infot.wait_for_publish()
        infot = mqttc.publish(topic["acc_y"], acc["y"], qos=2)
        infot.wait_for_publish()
        infot = mqttc.publish(topic["acc_z"], acc["z"], qos=2)
        infot.wait_for_publish()
        infot = mqttc.publish(topic["obstacle"], sonar.distance, qos=2)
        infot.wait_for_publish()
        infot = mqttc.publish(topic["brightness"], brightness, qos=2)
        infot.wait_for_publish()
        infot = mqttc.publish(topic["temperature"], bmp280.temperature, qos=2)
        infot.wait_for_publish()
        infot = mqttc.publish(topic["pressure"], bmp280.pressure, qos=2)
        infot.wait_for_publish()

        time.sleep(0.5)

    mqttc.disconnect()
    mqttc.loop_stop()

    ardusimple_port.close()