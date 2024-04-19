import board
import busio
import adafruit_bmp280 # adafruit-circuitpython-bmp280 from https://github.com/adafruit/Adafruit_CircuitPython_BMP280.git
import time

i2c = busio.I2C(board.SCL, board.SDA)
bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)
bmp280.sea_level_pressure = 1009.2 # pressure at sea level at current location

if __name__ == '__main__':
    print("Temp.: %0.1f °C" % bmp280.temperature)
    print("Pressure: %0.1f hPa" % bmp280.pressure)
    print("Altitude: %0.2f meters" % bmp280.altitude)