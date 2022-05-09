import math
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import time


def Servo_Initialize():
   i2c_bus = busio.I2C(SCL,SDA)
   pca = PCA9685(i2c_bus)
   pca.frequency = 100
   return pca

def Steering(pca, angle):
    if angle > 180:
        angle = 180
    if angle < 0:
        angle = 0
    duty = ((angle/180)*6553) + 6553
    pca.channels[0].duty_cycle = math.floor(duty)

pca = Servo_Initialize()
Steering(pca, 65)
