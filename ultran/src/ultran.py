#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def talker():
   pub = rospy.Publisher('topic_name', Float32, queue_size=1)
   rospy.init_node('node_name', anonymous = True)
   rate = rospy.Rate(10)
   import RPi.GPIO as GPIO
   import time
   GPIO.setwarnings(False)     # Ignore warning for now
   GPIO.setmode(GPIO.BOARD)    # Use physical pin numberin

   TRIG = 7   # define input and output pins
   ECHO = 11

   GPIO.setup(TRIG, GPIO.OUT)
   GPIO.output(TRIG, False)
   GPIO.setup(ECHO, GPIO.IN)

   time.sleep(.5)              # let the sensor initialize
   time_start = time.time()    
   # trigger a reading
   GPIO.output(TRIG, True)
   time.sleep(0.00001)
   GPIO.output(TRIG, False)

   while GPIO.input(ECHO) == 0:
       start_time = time.time()
   while GPIO.input(ECHO) == 1:
       end_time = time.time()
   time_end = time.time()
   total_distance = (end_time - start_time) * 34300
   hello_str= total_distance
   #rospy.loginfo(hello_str)
   print(hello_str)
   pub.publish(hello_str)
   rate.sleep()

if __name__ =='__main__':
   while True:
      try:
         talker()
      except rospy.ROSInterruptException:
         pass



#from picamera import PiCamera
#from time import sleep
#camera = PiCamera()
#camera.start_preview()
#sleep(5)
#camera.stop_preview()

