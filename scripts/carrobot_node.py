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

class CarRoBotNode(object):
    def __init__(self, integrate_num_samples=10):
        rospy.init_node('base')
        self._tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=100)
        rospy.Subscriber("/rpms", UInt16, self._rpm_callback, queue_size=5)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        self._pi = pigpio.pi()

        self._last_rpm = 0

        self._pid = PID(0.1, 0.20, 0.01, setpoint=240)

    def _rpm_callback(self, rpm):
        self._last_rpm = min(400.0, rpm.data)

    def _odom_callback(self, odom):
        self._tf_broadcaster.sendTransform((rospy.get_param("~base_lidar_x", -0.1),
                                            rospy.get_param("~base_lidar_y", 0.0),
                                            rospy.get_param("~base_lidar_z", 0.11)),
                                           (0., 0., 0., 1.),
                                           odom.header.stamp,
                                           "neato_laser", "base_link")

    def main(self):
        r = rospy.Rate(10)

        while not rospy.is_shutdown():

            if(rospy.get_param("~lidar_rpm_control", 1) == 1):
                control = max(min(1.0, self._pid(self._last_rpm) / 220.0), 0.0)
                self._pi.hardware_PWM(12, 100, control / 2.0 * 1000000)

            r.sleep()

        self.shutdown()

    def shutdown(self):
        self._pi.hardware_PWM(12, 100, 0)





if __name__ == '__main__':
    bot = CarRoBotNode()
    bot.main()
