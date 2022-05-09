#!/usr/bin/env python3
import rospy
#from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
import math
from board import SCL,SDA
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


def callback(data):
    data = data.data
    rospy.loginfo(rospy.get_caller_id() + "i heard %s", data)
    print("status:",data[0])
    print("angle:",data[1])
    pca = Servo_Initialize()
    Steering(pca, data[1])

   
def listener():
   rospy.init_node('listener_node', anonymous = True)
   #rospy.Subscriber('topic_name', Float32, callback)
   rospy.Subscriber('lidar', Float64MultiArray, callback)
   rospy.spin()

if __name__ =='__main__':
   while True:
      try:
         listener()
      except rospy.ROSInterruptException:
         pass
