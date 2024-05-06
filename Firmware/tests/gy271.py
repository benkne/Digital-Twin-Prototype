import py_qmc5883l # py_qmc5883l from https://github.com/RigacciOrg/py-qmc5883l.git
import time

temp_offset = 24

sensor = py_qmc5883l.QMC5883L()
sensor.declination = 5.3 # magnetic declination in Vienna as of May 2024 (https://www.zamg.ac.at/cms/de/geophysik/produkte-und-services-1/online-deklinationsrechner)
sensor.calibration = [[1.011353814274526, 0.0003112697421351132, 556.2405586464367], [0.00031126974213510974, 1.0000085335949689, 3395.165166454792], [0.0, 0.0, 1.0]]

while True:
    m = sensor.get_bearing()
    #m = sensor.get_temp()/100+temp_offset
    print(m)
    time.sleep(0.1)