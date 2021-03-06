#!/usr/bin/env python3
import rospy
#from std_msgs.msg import Float32
from std_msgs.msg import Float32

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

def Motor_Speed(pca, percent, channel = 11):
    pca.channels[channel].duty_cycle = math.floor(percent*65535)
    print(percent)

def callback(data):
    data = data.data

    pca = Servo_Initialize()
    Motor_StartUp(pca)

    print('')
    print('Changing Sppeds:')
    time.sleep(1)
    Motor_Speed(pca, .16, 11)
    time.sleep(1)
    Motor_Speed(pca, .15, 11)
    rospy.loginfo(rospy.get_caller_id() + "i heard %s", data)

def listener():
    rospy.init_node('listener_node', anonymous = True)
    #rospy.Subscriber('topic_name', Float32, callback)
    rospy.Subscriber('topic_name', Float32, callback)
    rospy.spin()

if __name__ =='__main__':
   while True:
      try:
         listener()
      except rospy.ROSInterruptException:
         pass



#from picamera import PiCamera
#from time import sleep
#camera = PiCamera()
#camera.start_preview()
#sleep(5)
#camera.stop_preview()

