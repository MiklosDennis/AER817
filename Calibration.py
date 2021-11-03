import time
import qwiic_icm20948
import math
import numpy as np
import random
IMU = qwiic_icm20948.QwiicIcm20948()

IMU.begin()
def calib(gyaw,filterMode=0):


    calb_gyro = [0,0,0]
    ga = [0,0,0]
    ax = []
    ay = []
    az = []
    gx = []
    gy = []
    gz = []
    mx = []
    my = []
    mz = []
    it = 1

    d2r = 180/math.pi
    b = 32768
    if IMU.dataReady():
        for i in range(8):
            IMU.getAgmt()
            ax.append(IMU.axRaw)
            ay.append(IMU.ayRaw)
            az.append(IMU.azRaw)
            gx.append(IMU.gxRaw)
            gy.append(IMU.gyRaw)
            if i > 0 :
                aa = np.average(gz)
                if abs(IMU.gzRaw) > int( aa * 1.25):
                    IMU.gzRaw = int(np.average(gz) * (1 + random.randint(-1,1)*random.randint(1,10)/10))
            gz.append(IMU.gzRaw)
            mx.append(IMU.mxRaw)
            my.append(IMU.myRaw)
            mz.append(IMU.mzRaw)


        vax = np.average(ax)
        vay = np.average(ay)
        vaz = np.average(az)
        vgx = np.average(gx)
        vgy = np.average(gy)
        vgz = int(np.average(gz) * 0.68)   
        vmx = np.average(mx)
        vmy = np.average(my)
        vmz = np.average(mz)

        accel = [vax,vay,vaz]
        gyro = [vgx,vgy,vgz]
        mag = [vmx,vmy,vmz]

        ax = []
        ay = []
        az = [] 
        gx = []
        gy = []
        gz = []
        mx = []
        my = []
        mz = []

    if gyro[2] < 300 and gyro[2] > -300:
        calb_gyro[2] = 0
    else:
        calb_gyro[2] = gyro[2]
            
    pitch = round(math.atan2(-accel[0],math.sqrt(math.pow(accel[1],2) + math.pow(accel[2],2))),1)
    roll = round(math.atan2(-accel[1],accel[2]),1)
    yaw = round(math.atan2(-mag[1]*math.cos(roll) + mag[2]*math.sin(roll), mag[0]*math.cos(pitch)\
                      +mag[1]*math.sin(roll)*math.sin(pitch)+mag[2]*math.cos(roll)*math.sin(pitch)) + math.pi,1)
    ypr = [roll,pitch,yaw]
    
    gyaw = round(gyaw + (calb_gyro[2]/b),3)
    #print(accel,'\n',gyro,'\n',mag,'\n', calb_gyro,'\n',ga,'\n',ypr,'\n')        
    return gyaw, ypr
