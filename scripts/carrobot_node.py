#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
import tf


class CarRoBotNode(object):
    def __init__(self, integrate_num_samples=10):
        rospy.init_node('base')
        self._tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber("/odom", Odometry, queue_size=100)

    def _odom_callback(self, odom):
        self._tf_broadcaster.sendTransform((0., 0., rospy.get_param('lidar_height')),
                                           (0., 0., 0., 0.),
                                           imu.header.stamp,
                                           "neato_laser", "base_link")

if __name__ == '__main__':
    pose = CarRoBotNode()
    rospy.spin()
