#!/usr?bin/env python
import math
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import time


def Servo_Initialize():
   i2c = busio.I2C(SCL,SDA)
   pca = PCA9685(i2c)
   pca.frequency = 100
   return pca

def Motor_StartUp(pca):
   print('Starting Motor Start Up Sequence')
   pca.channels[11].duty_cycle = math.floor(.15*65535)
   time.sleep(1)
   pca.channels[11].duty_cycle = math.floor(.2*65535)
   time.sleep(1)
   pca.channels[11].duty_cycle = math.floor(.15*65535)
   time.sleep(1)
   pca.channels[11].duty_cycle = math.floor(.1*65535)
   time.sleep(1)
   pca.channels[11].duty_cycle = math.floor(.15*65535)
   time.sleep(1)
   print('startup complete')

def Motor_Speed(pca, percent, channel = 3):
   pca.channels[channel].duty_cycle = math.floor(percent*65535)
   print(percent)

pca = Servo_Initialize()
#Motor_StartUp(pca)

#GPIO.setup(12, GPIO.IN)


for i in range(250):
    Motor_Speed(pca, 0.16, 3)
    time.sleep(0.3)
    Motor_Speed(pca, 0.15,3)
    time.sleep(1.5)
#while time.time() < endtime:
#   currunttime = time.time()
#   Motor_Speed(pca, .16, 11)



