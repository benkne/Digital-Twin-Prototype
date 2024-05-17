import time
from adafruit_servokit import ServoKit # adafruit-circuitpython-servokit from https://github.com/adafruit/Adafruit_CircuitPython_ServoKit.git

kit = ServoKit(channels=16)

while True:
    kit.servo[2].angle = 65
    time.sleep(1)
    kit.servo[2].angle = 30
    time.sleep(1)
    kit.servo[2].angle = 65
    time.sleep(1)
    kit.servo[2].angle = 100
    time.sleep(1)