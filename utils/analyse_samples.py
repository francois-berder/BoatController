#!/usr/bin/env python3
"""
Analyse samples collected from MPU6050.
Flash mpu6050_collect application on the board, let it run for a while.
Take out the SD card and copy MPU6050.TXT next to this script.
Run analysis: ./analyse_samples.py MPU6050.TXT
"""

import csv
import statistics
import sys

HEADERS = ['accel x', 'accel y', 'accel z', 'gyro x', 'gyro y', 'gyro z']
FORMAT_STR = '{0:15} | {1:20} | {2:20}'

if len(sys.argv) < 2:
    print('{} <path-to-sample-file>'.format(sys.argv[0]))
    exit(-1)

f = open(sys.argv[1])
reader = csv.reader(f)
rows = [list(map(int, r)) for r in reader]
columns = [list(tmp) for tmp in zip(*rows)]

# results = [NAME, MEAN, STDEV]
results = [()] * 6
for i, c in enumerate(columns):
    results[i] = (HEADERS[i], statistics.mean(c), statistics.stdev(c))

h = FORMAT_STR.format('name', 'average', 'standard deviation')
print(h)
print('-' * len(h))
for r in results:
    print(FORMAT_STR.format(r[0], r[1], r[2]))

# Take average of 4 samples, compute mean (should not change and) standard deviation
results = [()] * 6
ax = columns[0]
l = int(len(ax) / 4)
for i, c in enumerate(columns):
    s = [0] * l
    for j in range(l):
        s[j] = c[4 * j] + c[4 * j + 1] + c[4 * j + 2] + c[4 * j + 3]
        s[j] /= 4
    print(FORMAT_STR.format('smooth ' + HEADERS[i], statistics.mean(s), statistics.stdev(s)))
