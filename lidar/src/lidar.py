#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import math
import statistics
import numpy

def switch (lidar):
    front = lidar[0]
    left = lidar[1]-170
    right = lidar[2]-170
    if left < 0:
        left =0
    if right <0:
        right =0
    if left == 0:
        status = 2
    elif right == 0:
        status = 3;
    elif left == 0 and right == 0:
        status = 1
    else:
        status = 4
        
    return(status)

def controller(lidar):
    front = lidar[0]
    left = lidar[1]-170
    right = lidar[2]-170
    midpoint = (left+right)/2
    error = midpoint - left
    P = 0.5*error
    print("P:",P)
    if abs(left-right)<400:
        P = 65
    if P <45:
        P = 40+P*0.002
    if P >85:
        P = 85+0.002*P
    return(P)

def talker():
    pub = rospy.Publisher('lidar', Float64MultiArray, queue_size=10)
    rospy.init_node('node_name', anonymous = True)
    rate = rospy.Rate(10)
    from rplidar import RPLidar
    lidar = RPLidar('/dev/ttyUSB2')

    info = lidar.get_info()
#print(info)

    health = lidar.get_health()
#print(health)


    front = []
    right = []
    left = []
#print('%d: Got %d measurements' % (i, len(scan)))
    for i, scan in enumerate(lidar.iter_scans()):
        for j in enumerate(scan):
        #print("angle:",j[1][1],"distance",j[1][2])
            if j[1][1]<15 or j[1][1]>345:
                front.append(j[1][2])
            elif j[1][1]<285 and j[1][1]>255:
                left.append(j[1][2])
            elif j[1][1]<105 and j[1][1]>75:
                right.append(j[1][2])
        if i>0:
            break
    
    if front == []:
        front = 0.0
    else:
        front = min(front)
    if left == []:
        left = 0.0
    else:
        left = min(left)
    if right == []:
        right = 0.0
    else:
        right = min(right)

    print("Front:",front)
    print("right:",right)
    print("left:",left)
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    output = [front,right,left]
    status = switch(output)
    if status == 1:
        angle = 65
    elif status == 2:
        angle =  0
    elif status == 3:
        angle = 180
    elif status == 4:
        angle = controller(output)
    movement_data = Float64MultiArray()
    out = [status,angle ]
    print(out)
    movement_data.data = out
    pub.publish(movement_data)
    #pub.publish(output)

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

