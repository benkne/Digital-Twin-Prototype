import os
import time
import board
import busio
import pwmio
import threading
import subprocess
from datetime import datetime

import paho.mqtt.client as mqtt # paho-mqtt from https://github.com/eclipse/paho.mqtt.python.git
import adafruit_bmp280 # adafruit-circuitpython-bmp280 from https://github.com/adafruit/Adafruit_CircuitPython_BMP280.git
import adafruit_hcsr04 # adafruit-circuitpython-hcsr04 from https://github.com/adafruit/Adafruit_CircuitPython_HCSR04.git
import mpu6050 # mpu6050-raspberrypi from https://github.com/m-rtijn/mpu6050.git
import py_qmc5883l # py_qmc5883l from https://github.com/RigacciOrg/py-qmc5883l.git

import adafruit_ads1x15.ads1015 as ADS # adafruit-circuitpython-ads1x15 from https://github.com/adafruit/Adafruit_CircuitPython_ADS1x15.git
from adafruit_ads1x15.analog_in import AnalogIn

from ublox_gps import UbloxGps # sparkfun-ublox-gps from https://github.com/sparkfun/Qwiic_Ublox_Gps_Py.git
import serial

from adafruit_servokit import ServoKit # adafruit-circuitpython-servokit from https://github.com/adafruit/Adafruit_CircuitPython_ServoKit.git
from adafruit_motor import servo # adafruit-circuitpython-motor

mqtt_user = os.getenv('MQTT_USER')
mqtt_pass = os.getenv('MQTT_PASS')

mqtt_broker = "192.168.0.132"
mqtt_port = 1883

serial_port = "/dev/ttyACM0"
serial_baud = 115200

ntrip_user = os.getenv('APOS_USER')
ntrip_pass = os.getenv('APOS_PASS')

steer_angle_zero = 65
platform_angle_zero = 0

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
    "pressure" : "rover/environment/pressure",

    "move" : "rover/control/move",
    "steer" : "rover/control/steer",
    "platform" : "rover/control/platform",
}

gps_data = {
    "lon" : None,
    "lat" : None,
    "timestamp" : None
}

obstacle_distance = None

acc_calibration = {
    "x_corr" : -0.6,
    "y_corr" : -0.4,
    "z_corr" : 0.8
}

def speed(sp):
    p=sp/20+5
    duty = 65535 * p // 100
    print(duty)
    return duty

def message_recv(mqttc, obj, msg):
    message = str(msg.payload.decode("utf-8"))
    print(msg.topic + " " + str(msg.qos) + " " + message)

    if(msg.topic==topic["move"]):
        if(obstacle_distance==None or obstacle_distance>50): # only move if there is no obstacle within 50cm
            motor_pwm.duty_cycle = speed(13)
            time.sleep(0.3 + float(message)*0.1)
        motor_pwm.duty_cycle = speed(0)
    if(msg.topic==topic["steer"]):
        steer_servo.angle = steer_angle_zero + int(message)
    if(msg.topic==topic["platform"]):
        platform_servo.angle = platform_angle_zero + int((int(message)*180/45))


def mqtt_connect():
    mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqttc.on_message = message_recv
    mqttc.on_connect = lambda mqttc, obj, flags, reason_code, properties: (print("reason_code: " + str(reason_code)))
    mqttc.on_publish = lambda mqttc, obj, mid, reason_code, properties: (print("mid: " + str(mid)))
    mqttc.on_subscribe = lambda mqttc, obj, mid, reason_code_list, properties: (print("Subscribed: " + str(mid) + " " + str(reason_code_list)))

    # mqttc.on_log = lambda mqttc, obj, level, string: (print(string))

    mqttc.username_pw_set(username=mqtt_user,password=mqtt_pass)

    mqttc.connect(mqtt_broker, 1883, 60)

    mqttc.loop_start()

    return mqttc

def read_gps():
        try:
            while(True):
                coords = gps.geo_coords()
                gps_time = gps.date_time()
                gps_data["timestamp"] = datetime(gps_time.year, gps_time.month, gps_time.day, gps_time.hour, gps_time.min, gps_time.sec).strftime('%Y-%m-%dT%H:%M:%SZ')
                gps_data["lon"] = coords.lon
                gps_data["lat"] = coords.lat
        except (ValueError, IOError) as err:
            print(err)

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

    servos = ServoKit(channels=16)

    steer_servo = servos.servo[2]
    steer_servo.angle = steer_angle_zero

    platform_servo = servos.servo[1]
    platform_servo.angle = platform_angle_zero

    motor_pwm = pwmio.PWMOut(board.D13, duty_cycle=speed(0),  frequency=50)
    motor_pwm.duty_cycle = speed(0)

    # publish default values of control parameters
    infot = mqttc.publish(topic["move"], 0, qos=2)
    infot.wait_for_publish()
    infot = mqttc.publish(topic["steer"], 0, qos=2)
    infot.wait_for_publish()
    infot = mqttc.publish(topic["platform"], 0, qos=2)
    infot.wait_for_publish()

    
    mqttc.subscribe(topic["move"], qos=2)
    mqttc.subscribe(topic["steer"], qos=2)
    mqttc.subscribe(topic["platform"], qos=2)

    try:
        ardusimple_port = serial.Serial(serial_port, baudrate=serial_baud, timeout=1)
        gps = UbloxGps(ardusimple_port)

        ntripclient = [
            os.path.expanduser("/home/benkne/Documents/ntripclient/ntripclient"), # ntripclient from https://github.com/nunojpg/ntripclient.git
            "-s", "217.13.180.215",
            "-u", ntrip_user,
            "-p", ntrip_pass,
            "-r", "2201",
            "-D", serial_port,
            "-B", str(serial_baud),
            "-m", "APOS_Extended"
        ]
        ntrip_pid = subprocess.Popen(ntripclient, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("Ntrip Client has PID "+str(ntrip_pid.pid))

    except serial.SerialException as e:
        print("Ardusimple not connected!")
        gps = None

    if(gps!=None):
        gps_thread = threading.Thread(target=read_gps)
        gps_thread.start()

    while(True):
        ldr_resistance = (3.3-ldr.voltage)/ldr.voltage * 2200
        brightness = 12518931 * pow(ldr_resistance,-1.405) # Regression approximation from https://www.allaboutcircuits.com/projects/design-a-luxmeter-using-a-light-dependent-resistor/
    
        acc = mpu.get_accel_data()

        if(gps_data["timestamp"]!=None):
            infot = mqttc.publish(topic["timestamp"], gps_data["timestamp"], qos=2)
            infot.wait_for_publish()
        if(gps_data["lon"]!=None):
            infot = mqttc.publish(topic["lon"], round(gps_data["lon"], 6), qos=2)
            infot.wait_for_publish()
        if(gps_data["lat"]!=None):
            infot = mqttc.publish(topic["lat"], round(gps_data["lat"], 6), qos=2)
            infot.wait_for_publish()

        infot = mqttc.publish(topic["heading"], round(compass.get_bearing(), 1), qos=2)
        infot.wait_for_publish()
        infot = mqttc.publish(topic["acc_x"], round(acc["x"]+acc_calibration["x_corr"],1), qos=2)
        infot.wait_for_publish()
        infot = mqttc.publish(topic["acc_y"], round(acc["y"]+acc_calibration["y_corr"],1), qos=2)
        infot.wait_for_publish()
        infot = mqttc.publish(topic["acc_z"], round(acc["z"]+acc_calibration["z_corr"],1), qos=2)
        infot.wait_for_publish()
        try:
            obstacle_distance=round(sonar.distance, 1)
            infot = mqttc.publish(topic["obstacle"], obstacle_distance, qos=2)
            infot.wait_for_publish()
        except Exception as e:
            obstacle_distance=None
            print(e)
        infot = mqttc.publish(topic["brightness"], round(brightness, 1), qos=2)
        infot.wait_for_publish()
        infot = mqttc.publish(topic["temperature"], round(bmp280.temperature, 1), qos=2)
        infot.wait_for_publish()
        infot = mqttc.publish(topic["pressure"], round(bmp280.pressure, 1), qos=2)
        infot.wait_for_publish()

        time.sleep(0.5)

    gps_thread.join()

    mqttc.disconnect()
    mqttc.loop_stop()

    ardusimple_port.close()
