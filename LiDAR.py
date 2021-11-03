import serial
import time

ser = serial.Serial(port='COM12'\
                    ,baudrate = 115200\
                    ,parity=serial.PARITY_NONE\
                    ,stopbits=serial.STOPBITS_ONE\
                    ,bytesize=serial.EIGHTBITS\
                    ,timeout=1)

while True:
    time.sleep(0.5)
    count = ser.inWaiting()
    recv = ser.read(9)
    print(count)
    print(recv)
    if count > 8:
        recv = ser.read(9)
        ser.reset_input_buffer()
        print(recv)
        print('hi')
        if recv[0] == 0x59 and recv[1] == 0x59:
            distance = recv[2] + recv[3] * 256
            strength = recv[4] + recv[5] * 256
            print(distance, strength)
            ser.reset_input_buffer()
            print('ho')
    
