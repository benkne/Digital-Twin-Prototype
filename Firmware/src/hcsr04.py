import time
import board
import adafruit_hcsr04 # adafruit-circuitpython-hcsr04 from https://github.com/adafruit/Adafruit_CircuitPython_HCSR04.git

sonar = adafruit_hcsr04.HCSR04(trigger_pin=board.D23, echo_pin=board.D24)

if __name__ == '__main__':
    while True:
      try:
            print((sonar.distance))
      except RuntimeError:
            print("Retrying!")
      time.sleep(1)