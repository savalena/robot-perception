#!/usr/bin/python
# -*-coding: utf-8 -*-

import time
from pyiArduinoI2Cexpander import *
import numpy as np


class Motor(pyiArduinoI2Cexpander):
    def __init__(self, pin_left=2, pin_right=3):
        super(pyiArduinoI2Cexpander, self).__init__(0x08)
        self.V = 0.142  # m/sec
        self.w_right = -0.837  # rad/sec
        self.w_left =  0.68 #0.897
        self.LEFT_CONST = 1515
        self.RIGHT_CONST = 1415
        self.PIN_LEFT = pin_left
        self.PIN_RIGHT = pin_right
        self.scale = 1  # units / m
        self.pinMode(self.PIN_LEFT, OUTPUT, SERVO)
        self.pinMode(self.PIN_RIGHT, OUTPUT, SERVO)
        self.servoAttach(self.PIN_LEFT, 1300, 1590, -100, 100)
        self.servoAttach(self.PIN_RIGHT, 1300, 1590, -100, 100)

    def set_speed(self, om_left, om_right):
        """set speed on motors and run it"""
        analog_om_left = self.LEFT_CONST + om_left * 4
        analog_om_right = self.RIGHT_CONST - om_right * 4
        self.servoWriteMicroseconds(self.PIN_LEFT, analog_om_left)
        self.servoWriteMicroseconds(self.PIN_RIGHT, analog_om_right)

    def stop(self):
        self.servoWriteMicroseconds(self.PIN_LEFT, self.LEFT_CONST)
        self.servoWriteMicroseconds(self.PIN_RIGHT, self.RIGHT_CONST)

    def move_forward(self):
        print("move forward")
        self.set_speed(9.8, 10)

    def turn_left(self):
        print("move left")
        self.set_speed(0, 10)

    def turn_right(self):
        print("move right")
        self.set_speed(10, 0)

    def time_forward(self, meters=1):
        return meters / (self.V * self.scale)

    def time_right_turn(self, rads=3.14):
        return np.fabs(rads / self.w_right)

    def time_left_turn(self, rads=3.14):
        return np.fabs(rads / self.w_left)

if __name__ == '__main__':
    ### usage ###
    motors = Motor()
    motors.move_forward(0.7)
    motors.turn_left(3.14)
    motors.turn_right(3.14)
