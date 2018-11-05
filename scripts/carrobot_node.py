#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import time
import pigpio

class CarRoBotNode(object):
    def __init__(self, integrate_num_samples=10):
        rospy.init_node('base')
        self._tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=100)

        self._pi = pigpio.pi()
        self._pi.hardware_PWM(12, 100, min(1.0, rospy.get_param("~lidar_motor_pwm_duty_cycle", 0.9)) * 1000000.0)

    def _odom_callback(self, odom):
        self._tf_broadcaster.sendTransform((rospy.get_param("~base_lidar_x"),
                                            rospy.get_param("~base_lidar_y"),
                                            rospy.get_param("~base_lidar_z")),
                                           (0., 0., 0., 1.),
                                           odom.header.stamp,
                                           "neato_laser", "base_link")

    def main(self):
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            r.sleep()

        shutdown()

    def shutdown(self):
        self._pi.hardware_PWM(13, 100, 0)





if __name__ == '__main__':
    bot = CarRoBotNode()
    bot.main()
