#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
from simple_pid import PID
import tf
import time
import pigpio
import sys

rospy.init_node('test')
vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

r = rospy.Rate(1)

while not rospy.is_shutdown():

    t = Twist()
    t.angular.z = float(sys.argv[1])
    vel_pub.publish(t)

    r.sleep()

    t = Twist()
    t.angular.z = 0.0
    vel_pub.publish(t)

    r.sleep()
