import time
import board
import pwmio
from adafruit_motor import servo # adafruit-circuitpython-motor from https://github.com/adafruit/Adafruit_CircuitPython_Motor.git

dir(board)

# create a PWMOut object on Pin D5.
pwm = pwmio.PWMOut(board.D12, duty_cycle=2 ** 15,  frequency=100)

# Create a servo object.
servo = servo.Servo(pwm)


while True:
    for angle in range(0, 130, 5):
        servo.angle = angle
        print(angle)
        time.sleep(0.1)
    for angle in range(130, 0, -5):
        servo.angle = angle
        print(angle)
        time.sleep(0.1)
