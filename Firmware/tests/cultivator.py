import time
from adafruit_servokit import ServoKit # adafruit-circuitpython-servokit from https://github.com/adafruit/Adafruit_CircuitPython_ServoKit.git

kit = ServoKit(channels=16)

while True:
    kit.servo[1].angle = 0
    time.sleep(1)
    kit.servo[1].angle = 45
    time.sleep(1)
    kit.servo[1].angle = 90
    time.sleep(1)
    kit.servo[1].angle = 135
    time.sleep(1)
    kit.servo[1].angle = 180
    time.sleep(1)