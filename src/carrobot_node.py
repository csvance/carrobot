#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import ChannelFloat32
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

        self._transforms = {'tf_base_lidar': [float(c) for c in rospy.get_param("~tf_base_lidar").split(",")],
                            'tf_base_prox_left': [float(c) for c in rospy.get_param("~tf_base_prox_left").split(",")],
                            'tf_base_prox_right': [float(c) for c in rospy.get_param("~tf_base_prox_right").split(",")]
                            }

        self._lidar_rpm_control = rospy.get_param("~lidar_rpm_control", True)

        self._gpio_lidar_rpm = rospy.get_param("~gpio_lidar_pwm", 12)
        self._gpio_prox_left = rospy.get_param("~gpio_prox_left", 26)
        self._gpio_prox_right = rospy.get_param("~gpio_prox_right", 19)

        self._prox_left = 0
        self._prox_right = 0

        self._prox_dist = 0.2

        self.prox_left_pub = rospy.Publisher("prox_left", PointCloud, queue_size=5)
        self.prox_right_pub = rospy.Publisher("prox_right", PointCloud, queue_size=5)

        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=100)

        self._last_rpm = 0
        if self._lidar_rpm_control:
            rospy.Subscriber("/rpms", UInt16, self._rpm_callback, queue_size=5)

        self._pi = pigpio.pi()
        self._pi.set_mode(self._gpio_prox_left, pigpio.INPUT)
        self._pi.callback(self._gpio_prox_left, pigpio.EITHER_EDGE, self._gpio_callback)
        self._pi.set_mode(self._gpio_prox_right, pigpio.INPUT)
        self._pi.callback(self._gpio_prox_right, pigpio.EITHER_EDGE, self._gpio_callback)

        self._pid = PID(0.1, 0.20, 0.01, setpoint=240)

    def _gpio_callback(self, gpio, level, tick):

        # We only care about the sensor being triggered
        if level:
            return

        pcl = PointCloud()
        pcl.header.stamp = rospy.Time.now()

        intensities = []
        for i in range(-5, 6):
            point = Point32()
            point.x = self._prox_dist
            point.y = i / 100.0
            point.z = 0.0
            pcl.points.append(point)

            intensities.append(1.0)

        channel = ChannelFloat32()
        channel.name = "intensity"
        channel.values = intensities
        pcl.channels.append(channel)

        if gpio == self._gpio_prox_left and self._prox_left == 1:
            self._prox_left = 0

            self._tf_broadcaster.sendTransform(self._transforms['tf_base_prox_left'],
                                               (0., 0., 0., 1.),
                                               pcl.header.stamp,
                                               "prox_left", "base_link")

            pcl.header.frame_id = "prox_left"
            self.prox_left_pub.publish(pcl)

        elif gpio == self._gpio_prox_right and self._prox_right == 1:
            self._prox_right = 0

            self._tf_broadcaster.sendTransform(self._transforms['tf_base_prox_right'],
                                               (0., 0., 0., 1.),
                                               pcl.header.stamp,
                                               "prox_right", "base_link")

            pcl.header.frame_id = "prox_right"
            self.prox_right_pub.publish(pcl)

    def _rpm_callback(self, rpm):
        self._last_rpm = min(400.0, rpm.data)

    def _odom_callback(self, odom):
        self._tf_broadcaster.sendTransform(self._transforms['tf_base_lidar'],
                                           (0., 0., 0., 1.),
                                           odom.header.stamp,
                                           "neato_laser", "base_link")


    def handle_prox(self):

            pcl = PointCloud()
            pcl.header.stamp = rospy.Time.now()

            self._prox_left = self._pi.read(self._gpio_prox_left)
            if self._prox_left == 0:
                intensities = []
                for i in range(-5, 6):
                    point = Point32()
                    point.x = self._prox_dist
                    point.y = i / 100.0
                    point.z = 0.0
                    pcl.points.append(point)

                    intensities.append(1.0)

                channel = ChannelFloat32()
                channel.name = "intensity"
                channel.values = intensities
                pcl.channels.append(channel)

            self._tf_broadcaster.sendTransform(self._transforms['tf_base_prox_left'],
                                               (0., 0., 0., 1.),
                                               pcl.header.stamp,
                                               "prox_left", "base_link")

            pcl.header.frame_id = "prox_left"
            self.prox_left_pub.publish(pcl)

            pcl = PointCloud()
            pcl.header.stamp = rospy.Time.now()

            self._prox_right = self._pi.read(self._gpio_prox_right)
            if self._prox_right == 0:
                intensities = []
                for i in range(-5, 6):
                    point = Point32()
                    point.x = self._prox_dist
                    point.y = i / 100.0
                    point.z = 0.0
                    pcl.points.append(point)

                    intensities.append(1.0)

                channel = ChannelFloat32()
                channel.name = "intensity"
                channel.values = intensities
                pcl.channels.append(channel)

            self._tf_broadcaster.sendTransform(self._transforms['tf_base_prox_right'],
                                               (0., 0., 0., 1.),
                                               pcl.header.stamp,
                                               "prox_right", "base_link")

            pcl.header.frame_id = "prox_right"
            self.prox_right_pub.publish(pcl)


    def main(self):
        r = rospy.Rate(10)

        while not rospy.is_shutdown():

            if self._lidar_rpm_control:
                control = max(min(1.0, self._pid(self._last_rpm) / 220.0), 0.0)
                self._pi.hardware_PWM(self._gpio_lidar_rpm, 100, control / 2.0 * 1000000)

            self.handle_prox()

            r.sleep()

        self.shutdown()

    def shutdown(self):
        self._pi.hardware_PWM(self._gpio_lidar_rpm, 100, 0)





if __name__ == '__main__':
    bot = CarRoBotNode()
    bot.main()
