#! /usr/bin/env python
# -*- coding: utf-8 -*-

#Publishes message with required ID type and speed to topic to be sent via can.
#Software designed by Andrey Smirnov, Moscow State Unversity, 2022

import os
import time
import numpy as np
import rospy
from std_msgs.msg import UInt16MultiArray
import struct



rospy.init_node('CAN_Test')
pub = rospy.Publisher('CAN_Tx_Buffer', UInt16MultiArray, queue_size=1)
c = 0
while(True):
    RxBuffer_ROS = UInt16MultiArray()
    try:
        CAN_ID = int(input('CAN_ID: '))
        Message_Type = int(input('Message_Type: ' ))
        Motor_Speed = float(input('Motor_Speed: ' ))
        RxBuffer_ROS.data.append(CAN_ID)
        RxBuffer_ROS.data.append(Message_Type)
        arr = list(struct.pack("<f", Motor_Speed))
        print(arr)
        for i in arr:
            RxBuffer_ROS.data.append(int(i))
        pub.publish (RxBuffer_ROS)
        print("Published!")
    except ValueError:
        print("Not a number")
