#!/usr/bin/env python3
"""
Take a file filled with samples from MPU6050 and plot speed.
"""

import csv
import math
import sys
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print('{} <path-to-directory>'.format(sys.argv[0]))
    exit(-1)

f = open('{}/{}'.format(sys.argv[1], 'MPU6050.TXT')
reader = csv.reader(f)
rows = [list(map(float, r)) for r in reader]
columns = [list(tmp) for tmp in zip(*rows)]

steps = len(rows)

speed_x = [0.0] * steps
speed_y = [0.0] * steps
speed_z = [0.0] * steps

acc_x = columns[0]
acc_Y = columns[1]
acc_z = columns[2]

for i in range(1, steps):
    speed_x[i] = speed_x[i - 1] + acc_x[i]
    speed_y[i] = speed_y[i - 1] + acc_x[i]
    speed_z[i] = speed_z[i - 1] + acc_x[i]

plt.plot(speed_x, label='x')
plt.plot(speed_y, label='y')
plt.plot(speed_z, label='z')
plt.legend()
plt.show()
