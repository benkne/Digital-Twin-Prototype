from mpu6050 import mpu6050 # mpu6050-raspberrypi from https://github.com/m-rtijn/mpu6050.git

mpu = mpu6050(0x68)

if __name__ == '__main__':
    data = mpu.get_all_data()
    print(data)