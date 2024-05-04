import py_qmc5883l # py_qmc5883l from https://github.com/RigacciOrg/py-qmc5883l.git

sensor = py_qmc5883l.QMC5883L()
sensor.declination = 5.26
sensor.calibration = [[1.011353814274526, 0.0003112697421351132, 556.2405586464367], [0.00031126974213510974, 1.0000085335949689, 3395.165166454792], [0.0, 0.0, 1.0]]
m = sensor.get_bearing()
print(m)
