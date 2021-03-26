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

        # arrays for storing data  
        self.ranges_array = np.zeros((1, 1081))
        self.angles_array = np.zeros((1, 1081))
        self.delta_position_array = np.zeros((1, 2))

        self.new_deltas = np.zeros((1,2))
        self.th = 0

        self.motors = Motor()

        t = rospy.Time.now().to_sec()
        self.time = t
        self.start_time = t
        self.time_array = [t]
		self.is_scan_obtained = False

        self.delta_forward = t + self.motors.time_forward(meters=0.5)
        self.delta_left = self.motors.time_left_turn(rads=np.pi / 2) + self.delta_forward
        self.delta_forward2 = self.motors.time_forward(meters=0.5) + self.delta_left
        self.delta_left2 = self.motors.time_left_turn(rads=np.pi / 2) + self.delta_forward2
        self.delta_forward3 = self.motors.time_forward(meters=0.5) + self.delta_left2
        self.delta_left3 = self.motors.time_left_turn(rads=np.pi / 2) + self.delta_forward3
        self.delta_forward4 = self.motors.time_forward(meters=0.5) + self.delta_left3
        self.delta_left4 = self.motors.time_left_turn(rads=np.pi / 2) + self.delta_forward4

        scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1. / 10), self.move_and_scan)

    def scan_callback(self, data):
    	# obtain the scan
        self.scan = data	
        self.ranges, self.angles = self.cvt_ros_scan2points(data)
		self.is_scan_obtained = True

    @staticmethod
    def cvt_ros_scan2points(scan):
    	# parse ros scans in angle and ranges
        ranges = np.array(scan.ranges)
        n = ranges.shape[0]
        angles = np.arange(scan.angle_min, scan.angle_min + n * scan.angle_increment, scan.angle_increment)
        return ranges, angles

    def move_and_scan(self, event):

        # move forward
        if rospy.Time.now().to_sec() < self.delta_forward and self.is_scan_obtained:
            rospy.loginfo("MOVE FORWARD")
            self.motors.move_forward()

            delta = np.array([self.motors.V * (rospy.Time.now().to_sec() - self.time), 0])

            self.new_deltas = delta

            self.time = rospy.Time.now().to_sec()
            self.add_data()

        # move left
        if self.delta_left > rospy.Time.now().to_sec() > self.delta_forward:
            rospy.loginfo("MOVE LEFT")
            self.motors.turn_left()

            self.new_deltas = np.array([0, self.motors.w_left * (rospy.Time.now().to_sec() - self.time)])

            self.time = rospy.Time.now().to_sec()
            self.add_data()

        # move forward 2
        if self.delta_forward2 > rospy.Time.now().to_sec() > self.delta_left:
            rospy.loginfo("MOVE FORWARD 2")
            self.motors.move_forward()
	
            self.new_deltas = np.array([self.motors.V * (rospy.Time.now().to_sec() - self.time), 0])

            self.time = rospy.Time.now().to_sec()
            self.add_data()

        # move left 2
        if self.delta_left2 > rospy.Time.now().to_sec() > self.delta_forward2:
            rospy.loginfo("MOVE LEFT 2")
            self.motors.turn_left()

            self.new_deltas = np.array([0, self.motors.w_left * (rospy.Time.now().to_sec() - self.time)])

            self.time = rospy.Time.now().to_sec()
            self.add_data()

        # move forward 3
        if self.delta_forward3 > rospy.Time.now().to_sec() > self.delta_left2:
            rospy.loginfo("MOVE FORWARD 3")
            self.motors.move_forward()

            self.new_deltas = np.array([self.motors.V * (rospy.Time.now().to_sec() - self.time), 0])

            self.time = rospy.Time.now().to_sec()
            self.add_data()
	
	# left3
        if self.delta_left3 > rospy.Time.now().to_sec() > self.delta_forward3:
            rospy.loginfo("MOVE LEFT 3")
            self.motors.turn_left()

            self.new_deltas = np.array([0, self.motors.w_left * (rospy.Time.now().to_sec() - self.time)])

            self.time = rospy.Time.now().to_sec()
            self.add_data()

	#forward4
        if self.delta_forward4 > rospy.Time.now().to_sec() > self.delta_left3:
            rospy.loginfo("MOVE FORWARD 4")
            self.motors.move_forward()

            self.new_deltas  = np.array([self.motors.V * (rospy.Time.now().to_sec() - self.time), 0])

            self.time = rospy.Time.now().to_sec()
            self.add_data()

	# left4
        if self.delta_left4 > rospy.Time.now().to_sec() > self.delta_forward4:
            rospy.loginfo("MOVE LEFT 4")
            self.motors.turn_left()

            self.new_deltas = np.array([0, self.motors.w_left * (rospy.Time.now().to_sec() - self.time)])

            self.time = rospy.Time.now().to_sec()
            self.add_data()
        # save data

        if rospy.Time.now().to_sec() > self.delta_left4:
            self.motors.stop()
            self.timer.shutdown()

    def add_data(self):
    	# append data to the numpy array
        self.delta_position_array = np.append(self.delta_position_array,  self.new_deltas.reshape(
                (1, 2)), axis=0)
        self.ranges_array = np.append(self.ranges_array, self.ranges.reshape((1,1081)), axis=0)
        self.angles_array = np.append(self.angles_array, self.angles.reshape((1,1081)), axis=0)
        self.time_array.append(rospy.Time.now().to_sec())

    def save_data(self):
		# save data as npy
        self.time_array = np.array(np.array(self.time_array))
        np.save("/home/ubuntu/catkin_ws/src/robot-perception/src/position", self.delta_position_array)
        np.save("/home/ubuntu/catkin_ws/src/robot-perception/src/time", self.time_array)
        np.save("/home/ubuntu/catkin_ws/src/robot-perception/src/ranges", self.ranges_array)
        np.save("/home/ubuntu/catkin_ws/src/robot-perception/src/angles", self.angles_array)



if __name__ == '__main__':
    try:
        rospy.init_node('main_node', anonymous=True)
        main_node = Main()
        rospy.spin()
    except rospy.ROSInterruptException:
        self.motors.stop()
    