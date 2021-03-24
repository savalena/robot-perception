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

        N = 1000
        self.ranges_array = np.zeros((1, 1081))
        self.angles_array = np.zeros((1, 1081))
        self.delta_position_array = np.zeros((1, 3))

        self.new_deltas = np.zeros((1,3))
        self.th = 0

        self.motors = Motor()

        t = rospy.Time.now().to_sec()
        self.time = t
        self.start_time = t
        self.time_array = [t]
	self.flag = False

        self.delta_forward = t + self.motors.time_forward(meters=0.8)
        self.delta_left = self.motors.time_left_turn(rads=np.pi / 2) + self.delta_forward
        self.delta_forward2 = self.motors.time_forward(meters=0.2) + self.delta_left
        self.delta_left2 = self.motors.time_left_turn(rads=np.pi / 2) + self.delta_forward2
        self.delta_forward3 = self.motors.time_forward(meters=0.3) + self.delta_left2
        rospy.loginfo(self.motors.time_forward(meters=0.8))

        scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1. / 20), self.move_and_scan)

    def scan_callback(self, data):
        self.scan = data	
        self.ranges, self.angles = self.cvt_ros_scan2points(data)
	self.flag = True
        # rospy.loginfo("ranges %s", self.ranges)
        # rospy.loginfo("angles %s", self.angles)

    @staticmethod
    def cvt_ros_scan2points(scan):
        ranges = np.array(scan.ranges)
        n = ranges.shape[0]
        angles = np.arange(scan.angle_min, scan.angle_min + n * scan.angle_increment, scan.angle_increment)
        # x = ranges * np.cos(angles)
        # y = ranges * np.sin(angles)
        # plt.plot(x, y, '.')
        # plt.savefig('/home/ubuntu/catkin_ws/src/robot-perception/src/scan_view.png')
        return ranges, angles

    def move_and_scan(self, event):
        rospy.loginfo("current time %s", rospy.Time.now().to_sec())
        rospy.loginfo("delta forward %s", self.delta_forward)

        # move forward
        if rospy.Time.now().to_sec() < self.delta_forward and self.flag:
            rospy.loginfo("MOVE FORWARD")
            self.motors.move_forward(self.delta_forward)

            delta = np.array([self.motors.V * (rospy.Time.now().to_sec() - self.time), 0])
            dx = (delta[0]) * np.cos(self.th)
            dy = (delta[0]) * np.sin(self.th)
            dth = delta[1]
            self.th += delta[1]
            self.new_deltas = np.array([dx, dy, dth])

            self.time = rospy.Time.now().to_sec()
            self.add_data()

        # move left
        if self.delta_left > rospy.Time.now().to_sec() > self.delta_forward:
            rospy.loginfo("MOVE LEFT")
            self.motors.turn_left(self.delta_left)
            delta = np.array([0, self.motors.w_left * (rospy.Time.now().to_sec() - self.time)])

            dx = (delta[0]) * np.cos(self.th)
            dy = (delta[0]) * np.sin(self.th)
            dth = delta[1]
            self.th += delta[1]
            self.new_deltas = np.array([dx, dy, dth])

            self.time = rospy.Time.now().to_sec()
            self.add_data()

        # move forward 2
        if self.delta_forward2 > rospy.Time.now().to_sec() > self.delta_left:
            rospy.loginfo("MOVE FORWARD 2")
            delta = np.array([self.motors.V * (rospy.Time.now().to_sec() - self.time), 0])

            dx = (delta[0]) * np.cos(self.th)
            dy = (delta[0]) * np.sin(self.th)
            dth = delta[1]
            self.th += delta[1]
            self.new_deltas = np.array([dx, dy, dth])

            self.time = rospy.Time.now().to_sec()
            self.add_data()

        # move left 2
        if self.delta_left2 > rospy.Time.now().to_sec() > self.delta_forward2:
            rospy.loginfo("MOVE LEFT 2")
            self.motors.turn_left(self.delta_forward2)

            delta = np.array([0, self.motors.w_left * (rospy.Time.now().to_sec() - self.time)])
            dx = (delta[0]) * np.cos(self.th)
            dy = (delta[0]) * np.sin(self.th)
            dth = delta[1]
            self.th += delta[1]
            self.new_deltas = np.array([dx, dy, dth])

            self.time = rospy.Time.now().to_sec()
            self.add_data()

        # move forward 3
        if self.delta_forward3 > rospy.Time.now().to_sec() > self.delta_left2:
            rospy.loginfo("MOVE FORWARD 3")
            self.motors.move_forward(self.delta_forward2)

            delta = np.array([self.motors.V * (rospy.Time.now().to_sec() - self.time), 0])
            dx = (delta[0]) * np.cos(self.th)
            dy = (delta[0]) * np.sin(self.th)
            dth = delta[1]
            self.th += delta[1]
            self.new_deltas = np.array([dx, dy, dth])

            self.time = rospy.Time.now().to_sec()
            self.add_data()

        # save data
        if rospy.Time.now().to_sec() > self.delta_forward3:
            self.motors.stop()
            self.save_data()

    def add_data(self):
        self.delta_position_array = np.append(self.delta_position_array,  self.new_deltas.reshape(
                (1, 3)), axis=0)

        rospy.loginfo("1: %s", self.ranges_array.shape)
        rospy.loginfo("2: %s", self.ranges.shape)
        self.ranges_array = np.append(self.ranges_array, self.ranges.reshape((1,1081)), axis=0)
        self.angles_array = np.append(self.angles_array, self.angles.reshape((1,1081)), axis=0)
        self.time_array.append(rospy.Time.now().to_sec())

    def save_data(self):
	rospy.loginfo("shape of %s", self.ranges_array)
        self.time_array = np.array(np.array(self.time_array))
        np.save("/home/ubuntu/catkin_ws/src/robot-perception/src/position", self.delta_position_array)
        np.save("/home/ubuntu/catkin_ws/src/robot-perception/src/time", self.time_array)
        np.save("/home/ubuntu/catkin_ws/src/robot-perception/src/ranges", self.ranges_array)
        np.save("/home/ubuntu/catkin_ws/src/robot-perception/src/angles", self.angles_array)


if __name__ == '__main__':
    try:
        rospy.init_node('node', anonymous=True)
        pf_node = Main()
        rospy.spin()
    except rospy.ROSInterruptException:
        self.motors.stop()
        self.save_data()
