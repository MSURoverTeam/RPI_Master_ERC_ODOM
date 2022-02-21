#!/usr/bin/env python3


#This script receives wheel odom data in rad/s via ros /Odom_Raw topic
#and uses it and IMU data to publish /odom topic for navigation
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float32
import board

import time
import board
import adafruit_bno055
import os
    
WHEEL_RADIUS = 0.055    # for computing speed


# current coordinates are computed automatically
#Theese coodinates are in an absolute coordinate system (fixed to map)
x = 0.0
y = 0.0



#andgle is received from IMU Sensor
th = 0.0

#velocities are received from wheel encoders
vx = 0.1    #we'll only need this one as our rover has tank-like steering
vy = 0.0    # theese velocities are in a coordinate system fixed to our rover
vth = 0.0

#time between speed measurements
dt = 0.05


def Subscriber_callback(Wheel_Odom):
    current_time = rospy.Time.now()
    global dt
    global sensor
    global x
    global y
    global th
    vx = WHEEL_RADIUS * Wheel_Odom.data
    th = sensor.euler[0]
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    
    x += delta_x
    y += delta_y
    th += delta_th

    odom_quat = sensor.quaternion

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    return



if(__name__ == '__main__'):
    #Connect to IMU Sensor
    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)

    rospy.init_node('odometry_publisher')

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    # Wheel linear speed in rad/s is received via odom_raw topic
    odom_raw_sub = rospy.Subscriber("Odom_Raw", Float32, Subscriber_callback)
    while True:
        time.sleep(0.005)
S
