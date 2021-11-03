import RPi.GPIO as GPIO
import time

pin = 17

GPIO.setmode(GPIO.BCM)

GPIO.setup(pin,GPIO.OUT)

while True:
    GPIO.output(pin,GPIO.HIGH)
    time.sleep(1)
    GPIO.output(pin,GPIO.LOW)
    time.sleep(1)
