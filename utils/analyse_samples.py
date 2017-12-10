#!/usr/bin/env python3
"""Analyse samples collected from MPU6050."""

import csv
import statistics
import sys

HEADERS = ['accel x', 'accel y', 'accel z', 'gyro x', 'gyro y', 'gyro z']
FORMAT_STR = '{0:10} | {1:20} | {2:20}'

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

print(FORMAT_STR.format('name', 'average', 'standard deviation'))
for r in results:
    print(FORMAT_STR.format(r[0], r[1], r[2]))
