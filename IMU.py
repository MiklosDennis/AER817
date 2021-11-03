import qwiic_icm20948
import math
import time

IMU = qwiic_icm20948.QwiicIcm20948()

IMU.begin()

d2r = 180/math.pi

while True:
    if IMU.dataReady():
        IMU.getAgmt()

        # Calculate Angles
        pitch = math.atan2(- IMU.axRaw,math.sqrt(math.pow(IMU.ayRaw,2) + math.pow(IMU.azRaw,2)))
        roll = math.atan2( -IMU.ayRaw,IMU.azRaw)
        yaw = math.atan2(-IMU.myRaw*math.cos(roll) + IMU.mzRaw*math.sin(roll), IMU.mxRaw*math.cos(pitch)\
                         +IMU.myRaw*math.sin(roll)*math.sin(pitch)+IMU.mzRaw*math.cos(roll)*math.sin(pitch))
        print('P:{:.2f}'.format(pitch*d2r)\
              , 'R:{:.2f}'.format(roll*d2r)\
              , 'Y:{:.2f}'.format(yaw*d2r+180))

        time.sleep(0.1)
