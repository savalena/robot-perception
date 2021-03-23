#!/usr/bin/python
# -*-coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt

def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
# ranges = np.load('/home/ubuntu/catkin_ws/src/robot-perception/src/ranges.npy')
# angles = np.load('/home/ubuntu/catkin_ws/src/robot-perception/src/angles.npy')
# x = ranges[0, :] * np.cos(angles[0, :])
# y = ranges[0, :] * np.sin(angles[0, :])
# plt.plot(x, y, '.')
# plt.savefig('/home/ubuntu/catkin_ws/src/robot-perception/src/scan_0idx.png')

fig, ax = plt.subplots()
# delta_pos = np.load('/home/ubuntu/catkin_ws/src/robot-perception/src/position.npy')
delta_pos = np.load('/home/alena/robot-perception/src/position.npy')
coords_x = [delta_pos[0, 0]]
coords_y = [delta_pos[0, 1]]
coords_th = [delta_pos[0, 2]]
th = 0
x = 0
y = 0
for i in range(1, delta_pos.shape[0]):
    x += (delta_pos[i, 0] + delta_pos[i - 1, 0]) * np.cos(coords_th[-1])
    y += (delta_pos[i, 0] + delta_pos[i - 1, 0]) * np.sin(coords_th[-1])
    th += delta_pos[i, 2] + delta_pos[i - 1, 2]
    if i == 28:
        print(1)
    coords_x.append(x)
    coords_y.append(y)
    coords_th.append(wrap_angle(th))

x = np.array(coords_x)
y = np.array(coords_y)
th = wrap_angle(np.array(coords_th))
ax.plot(coords_x, coords_y, '.')
# fig.savefig('/home/ubuntu/catkin_ws/src/robot-perception/src/trajectory.png')
plt.show()