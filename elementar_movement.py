#!/usr/bin/python
# -*-coding: utf-8 -*-

from kinematics import Motor, MobilePlatform

import numpy as np
import time

# init classes
robot = MobilePlatform()
motors = Motor()

# move forward
#print("move forward")
#om_l = 10
#om_r = 10
#motors.set_speed(om_l, om_r)
#time.sleep(2)

#print("stop")
#motors.stop()

# rotate left
print("move left")
#om_l, om_r = robot.left_rotation(1)
om_l = 0
om_r = 10
print(om_l, om_r)
motors.set_speed(om_l, om_r)
time.sleep(2)

# rotate roght
print("move right")
#om_l, om_r = robot.left_rotation(1)
om_l = 10
om_r = 0
print(om_l, om_r)
motors.set_speed(om_l, om_r)
time.sleep(2)

print("stop")
motors.stop()