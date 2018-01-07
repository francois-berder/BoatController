#!/usr/bin/env python3
"""
Analyse data collected when running firmware application.

Give the path to a directory containing OUTPUT.TXT, RADIO.TXT and MPU6050.TXT
Run analysis: ./analyse_samples.py <out>
"""

import csv
import sys
import matplotlib.pyplot as plt


def analyse_radio_data(indir):
    """
    It is expected that radio data is in RADIO.TXT in the following format:
    <ticks>, <dir>, <speed>
    <ticks>, <dir>, <speed>
    ...
    """
    f = open('{}/RADIO.TXT'.format(indir))
    reader = csv.reader(f)
    rows = [list(map(int, r)) for r in reader]
    columns = [list(tmp) for tmp in zip(*rows)]

    ticks = columns[0]
    t = [float(t) * 0.001 for t in ticks]
    dir_data = columns[1]
    speed_data = columns[2]

    dir_plot = plt.subplot(2, 1, 1)
    dir_plot.set_title('direction')
    dir_plot.set_ylim([3500,8500])

    dir_plot.plot(t, dir_data, label='direction')

    speed_plot = plt.subplot(2, 1, 2)
    speed_plot.set_title('speed')
    speed_plot.set_ylim([3500,8500])
    speed_plot.plot(t, speed_data, label='speed')

    plt.xlabel('time (s)')
    plt.savefig('{}/radio.png'.format(indir))

def analyse_output_data(indir):
    """
    It is expected that output data is in OUTPUT.TXT in the following format:
    <ticks>, <left_rudder>, <right_rudder>, <left_motor>, <right_motor>
    <ticks>, <left_rudder>, <right_rudder>, <left_motor>, <right_motor>
    ...
    """
    f = open('{}/OUTPUT.TXT'.format(indir))
    reader = csv.reader(f)
    rows = [list(map(int, r)) for r in reader]
    columns = [list(tmp) for tmp in zip(*rows)]

    ticks = columns[0]
    t = [float(t) * 0.001 for t in ticks]
    left_rudder_data = columns[1]
    right_rudder_data = columns[2]
    left_motor_data = columns[3]
    right_motor_data = columns[4]

    left_rudder_plot = plt.subplot(4, 1, 1)
    left_rudder_plot.set_title('left rudder')
    left_rudder_plot.set_ylim([3500, 8500])
    left_rudder_plot.plot(t, left_rudder_data, label='left_rudder')

    right_rudder_plot = plt.subplot(4, 1, 2)
    right_rudder_plot.set_title('right rudder')
    right_rudder_plot.set_ylim([3500, 8500])
    right_rudder_plot.plot(t, right_rudder_data, label='right_rudder')

    left_motor_plot = plt.subplot(4, 1, 3)
    left_motor_plot.set_title('left motor')
    left_motor_plot.set_ylim([3500, 8500])
    left_motor_plot.plot(t, left_motor_data, label='left_motor')

    right_motor_plot = plt.subplot(4, 1, 4)
    right_motor_plot.set_title('right motor')
    right_motor_plot.set_ylim([3500, 8500])
    right_motor_plot.plot(t, right_motor_data, label='right_motor')

    plt.tight_layout()
    plt.xlabel('time (s)')
    plt.savefig('{}/output.png'.format(indir))

def analyse_mpu6050_data(indir):
    """
    It is expected that output data is in OUTPUT.TXT in the following format:
    <ticks>, <left_rudder>, <right_rudder>, <left_motor>, <right_motor>
    <ticks>, <left_rudder>, <right_rudder>, <left_motor>, <right_motor>
    ...
    """
    f = open('{}/MPU6050.TXT'.format(indir))
    reader = csv.reader(f)
    rows = [list(map(int, r)) for r in reader]
    columns = [list(tmp) for tmp in zip(*rows)]

    ticks = columns[0]
    t = [float(t) * 0.001 for t in ticks]
    accel_x_data = columns[1]
    accel_y_data = columns[2]
    accel_z_data = columns[3]
    gyro_x_data = columns[4]
    gyro_y_data = columns[5]
    gyro_z_data = columns[6]

    # Plot raw accel data
    plt.figure(1)
    accel_x_plot = plt.subplot(3, 1, 1)
    accel_x_plot.set_title('accel x')
    accel_x_plot.plot(t, accel_x_data, label='accel_x')

    accel_y_plot = plt.subplot(3, 1, 2)
    accel_y_plot.set_title('accel y')
    accel_y_plot.plot(t, accel_y_data, label='accel_y')

    accel_z_plot = plt.subplot(3, 1, 3)
    accel_z_plot.set_title('accel z')
    accel_z_plot.plot(t, accel_z_data, label='accel_z')

    plt.tight_layout()
    plt.xlabel('time (s)')
    plt.savefig('{}/accel.png'.format(indir))

    # Plot raw gyro data
    plt.figure(2)
    gyro_x_plot = plt.subplot(3, 1, 1)
    gyro_x_plot.set_title('gyro x')
    gyro_x_plot.plot(t, gyro_x_data, label='gyro_x')

    gyro_y_plot = plt.subplot(3, 1, 2)
    gyro_y_plot.set_title('gyro y')
    gyro_y_plot.plot(t, gyro_y_data, label='gyro_y')

    gyro_z_plot = plt.subplot(3, 1, 3)
    gyro_z_plot.set_title('gyro z')
    gyro_z_plot.plot(t, gyro_z_data, label='gyro_z')

    plt.tight_layout()
    plt.xlabel('time (s)')
    plt.savefig('{}/gyro.png'.format(indir))

    # Plot angles (no filtering)
    t.insert(0, t[0])
    n = len(rows)
    pitch_data = [0.0] * (n + 1)
    roll_data = [0.0] * (n + 1)
    heading_data = [0.0] * (n + 1)

    for i in range(0, n):
        pitch_data[i + 1] = pitch_data[i] + gyro_x_data[i]
        roll_data[i + 1] = roll_data[i] + gyro_y_data[i]
        heading_data[i + 1] = heading_data[i] + gyro_z_data[i]

    plt.figure(3)
    roll_plot = plt.subplot(3, 1, 1)
    roll_plot.set_title('roll')
    roll_plot.plot(t, roll_data, label='roll')

    pitch_plot = plt.subplot(3, 1, 2)
    pitch_plot.set_title('pitch')
    pitch_plot.plot(t, pitch_data, label='pitch')

    heading_plot = plt.subplot(3, 1, 3)
    heading_plot.set_title('heading')
    heading_plot.plot(t, heading_data, label='heading')

    plt.tight_layout()
    plt.xlabel('time (s)')
    plt.savefig('{}/angles.png'.format(indir))

if len(sys.argv) != 2:
    print("Wrong number of arguments.")
    print(__doc__)
    exit(1)

indir = sys.argv[1]

analyse_radio_data(indir)
analyse_output_data(indir)
analyse_mpu6050_data(indir)
