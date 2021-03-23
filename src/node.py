#!/usr/bin/python
# -*-coding: utf-8 -*-

import rospy
import numpy as np
from kinematics import Motor
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt


class Main(object):
    def __init__(self):
        self.scan = None
        self.ranges = None
        self.angles = None
        scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

    def scan_callback(self, data):
        self.scan = data
        self.ranges, self.angles = self.cvt_ros_scan2points(data)
        rospy.loginfo("ranges %s", self.ranges.shape)
        rospy.loginfo("angles %s", self.angles.shape)

    @staticmethod
    def cvt_ros_scan2points(scan):
        ranges = np.array(scan.ranges)
        n = ranges.shape[0]
        angles = np.arange(scan.angle_min, scan.angle_min + n * scan.angle_increment, scan.angle_increment)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        #plt.plot(x, y, '.')
        #plt.savefig('/home/ubuntu/catkin_ws/src/robot-perception/src/scan_view.png')
        return ranges, angles


if __name__ == '__main__':
    try:
        rospy.init_node('node', anonymous=True)
        pf_node = Main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
