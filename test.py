import serial
import time
import math
import socket
import Calibration
import RPi.GPIO as GPIO
import sys
from gpiozero import Servo

# Setup Parameters
useServer = 0
hideOutput = 0 
if len(sys.argv) > 1:
    for arg in sys.argv:
        if arg == '-useServer':
            useServer = 1
        elif arg == '-hideOutput':
            hideOutput = 1

# Board Setup
servo = Servo(13)
val = 0
pin = 11
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin,GPIO.OUT)
GPIO.output(pin,0)
GPIO.setwarnings(False)

# Server
if useServer:
    TCP_IP = '192.168.2.39'
    TCP_PORT = 5045
    BUFFER_SIZE = 32

    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    
    try:
        s.connect((TCP_IP,TCP_PORT))
    except:
        print('Connection Failed')
    
    # If succesful connection turn on led
    GPIO.output(pin,1)
    
# Serial Port Initialization
ser = serial.Serial(port='/dev/ttyS0'\
                    ,baudrate = 115200)

def read_data():
    count = 0
    distance = 0
    d2r = 180/math.pi
    gyaw = 0
    while True:
        time.sleep(0.5)
        servo.value = val
        val = val + 0.1
        counter = ser.inWaiting()
        if counter > 8:
            bytes_serial = ser.read(9)
            ser.reset_input_buffer()

            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                distance = bytes_serial[2] + bytes_serial[3] * 256
                ser.reset_input_buffer()

        gyaw,rpy = Calibration.calib(gyaw,0)
            
        # Calculate coordinates
        if distance > 500:
            continue
        
        if not hideOutput:
            print('P{:.2f},'.format(rpy[1]*d2r)\
                  +'R{:.2f},'.format(rpy[0]*d2r)\
                  +'Y{:.2f}'.format(gyaw*d2r))
        
        x=round(distance * math.sin(gyaw)*math.cos(rpy[0]),1)
        y=round(distance * math.cos(gyaw)*math.cos(rpy[0]),1)
        z=round(distance * math.sin(rpy[0]),1)

        if count > 10000:
            break
        else:
            count = count +1

        text = '{:.2f},'.format(x)\
                  +'{:.2f},'.format(y)\
                  +'{:.2f}'.format(z)
        text = '{:<32s}'.format(text)
        #text = str(x)+','+str(y)+','+str(z) 
        
        if useServer:
            try:
                s.send(bytes(text,'utf-8'))
            except:
                print('Interuppted Connection')
                GPIO.output(pin,0)
                break

            

read_data()
