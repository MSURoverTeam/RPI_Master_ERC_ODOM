#!/usr/bin/env python3

#interprets byte array received from one of the wheels into rad/s
#odometry message
#Software designed by Andrey Smirnov, Moscow State Unversity, 2022

import os
import time
import numpy as np
import rospy
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Float32
import struct

# all CAN IDs of motors should be in the list
motor_CAN_IDs = [10,]
t = time.time()
rospy.init_node("Odom_Decoder")


def Subscriber_callback(RxBuffer_ROS):
    global pub
    global motor_CAN_IDs
    if int(RxBuffer_ROS.data[0]) in motor_CAN_IDs:
        out_msg = Float32()
        msg = list(RxBuffer_ROS.data[1:5:])
        speed = struct.unpack("<f", bytes(msg))[0]
        out_msg.data = speed
        pub.publish(out_msg)
 #   print(f'{speed:.14f}')
    return

def Publisher():
    print("Publisher Started!")
    global pub
    pub = rospy.Publisher('Odom_Raw', Float32, queue_size=1000)

def Subscriber():
    print( "Subscriber Started!")
    sub = rospy.Subscriber('CAN_Rx_Buffer', UInt16MultiArray, Subscriber_callback)
    rospy.spin()
    
if __name__ == "__main__":
    try:
        Publisher()
        Subscriber()
    except KeyboardInterrupt:
        None
