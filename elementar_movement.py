#!/usr/bin/python
# -*-coding: utf-8 -*-
from kinematics import Motor, MobilePlatform

import numpy as np
import time

if __name__ == 'main':
    # init classes
    robot = MobilePlatform()
    motors = Motor()

    # move forward
    om_l, om_r = robot.inverted_kin(robot.FULL_SPEED, 0)
    motors.set_speed(om_l, om_r)
    time.sleep(2)

    motors.stop()

    # rotate left
    # om_l, om_r = robot.left_rotation(0.1)
    # motors.set_speed(om_l, om_r)
    # time.sleep(2)

    # rotate right
    # om_l, om_r = robot.right_rotation(0.1)
    # motors.set_speed(om_l, om_r)
    # time.sleep(2)