from ublox_gps import UbloxGps # sparkfun-ublox-gps from https://github.com/sparkfun/Qwiic_Ublox_Gps_Py.git
import serial

port = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
gps = UbloxGps(port)

def run():
  
  try: 
    print("Listenting for UBX Messages.")
    while True:
      try: 
        coords = gps.geo_coords()
        print(coords.lat,coords.lon)
        #print("Heading of Motion: ", coords.headMot)

        #print(gps.stream_nmea())

        gps_time = gps.date_time()
        print("{}/{}/{}".format(gps_time.day, gps_time.month, gps_time.year))
        print("UTC Time {}:{}:{}".format(gps_time.hour, gps_time.min, gps_time.sec))
        #print("Valid date:{}\nValid Time:{}".format(gps_time.valid.validDate, gps_time.valid.validTime))

      except (ValueError, IOError) as err:
        print(err)
  
  finally:
    port.close()

if __name__ == '__main__':
  run()