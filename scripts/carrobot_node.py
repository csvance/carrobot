#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import time

class CarRoBotNode(object):
    def __init__(self, integrate_num_samples=10):
        rospy.init_node('base')
        self._tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber("/odom", Odometry, queue_size=100)

    def _scan_callback(self, scan):
        pass

    def _odom_callback(self, odom):
        self._tf_broadcaster.sendTransform((rospy.get_param('base_lidar_x'),
                                            rospy.get_param('base_lidar_y'),
                                            rospy.get_param('base_lidar_z')),
                                           (0., 0., 0., 1.),
                                           odom.header.stamp,
                                           "neato_laser", "base_link")

if __name__ == '__main__':
    bot = CarRoBotNode()
    rospy.spin()
