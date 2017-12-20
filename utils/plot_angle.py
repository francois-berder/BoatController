#!/usr/bin/env python3
"""
Take a file filled with samples and compute roll, pitch and yaw angles.
"""

import csv
import math
import sys
import matplotlib.pyplot as plt

# Sample is retrieved every 20 ms
DT = 0.02
GYROSCOPE_SENSITIVITY = 65.5
alpha = 0.98

if len(sys.argv) < 2:
    print('{} <path-to-sample-file>'.format(sys.argv[0]))
    exit(-1)

f = open(sys.argv[1])
reader = csv.reader(f)
rows = [list(map(float, r)) for r in reader]
columns = [list(tmp) for tmp in zip(*rows)]

steps = len(rows)

roll = [0.0] * steps
pitch = [0.0] * steps
yaw = [0.0] * steps

for i in range(1, steps):
    gyro_x = (columns[3][i] / GYROSCOPE_SENSITIVITY) * DT
    gyro_y = (columns[4][i] / GYROSCOPE_SENSITIVITY) * DT
    gyro_z = (columns[5][i] / GYROSCOPE_SENSITIVITY) * DT

    acc_x = math.atan2(columns[0][i], columns[2][i]) * 180.0 / math.pi
    acc_y = math.atan2(columns[1][i], columns[2][i]) * 180.0 / math.pi
    acc_z = math.atan2(columns[0][i], columns[1][i]) * 180.0 / math.pi

    roll[i] = alpha * (roll[i - 1] + gyro_x) - (1.0 - alpha) * acc_x
    pitch[i] = alpha * (pitch[i - 1] + gyro_y) + (1.0 - alpha) * acc_y
    yaw[i] = alpha * (yaw[i - 1] + gyro_z) #+ (1.0 - alpha) * acc_z

plt.plot(roll, label='roll')
plt.plot(pitch, label='pitch')
plt.plot(yaw, label='yaw')
plt.legend()
plt.show()
