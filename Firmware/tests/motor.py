import time
import board
import pwmio
import numpy
from adafruit_motor import servo # adafruit-circuitpython-motor from https://github.com/adafruit/Adafruit_CircuitPython_Motor.git

def speed(sp):
    p=sp/20+5
    duty = 65535 * p // 100
    print(duty)
    return duty


# create a PWMOut object on Pin D5.
pwm = pwmio.PWMOut(board.D13, duty_cycle=speed(0),  frequency=50)

pwm.duty_cycle = speed(0)
time.sleep(2)

sp = 0

cycle=50
period=1/2
while True:
    pwm.duty_cycle = speed(11)
    time.sleep(0.3)
    pwm.duty_cycle = speed(0)
    time.sleep(2)

while True:
    ins = input()
    if(ins=='s'):
        sp=0
    elif(ins=='w'):
        sp+=1
    else:
        sp-=1
    print(sp)
    pwm.duty_cycle = speed(sp)


# Create a servo object.
servo = servo.Servo(pwm)

while True:
    for angle in range(0, 130, 5):
        servo.angle = angle
        print(angle)
        time.sleep(0.3)
    for angle in range(130, 0, -5):
        servo.angle = angle
        print(angle)
        time.sleep(0.3)
