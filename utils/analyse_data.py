#!/usr/bin/env python3
"""
Analyse data collected when running firmware application.

Give the path to a directory containing OUTPUT.TXT, RADIO.TXT and MPU6050.TXT
Run analysis: ./analyse_samples.py <out>
"""

import csv
import math
import sys
import matplotlib.pyplot as plt

GYRO_SENSITIVITY = 65.5
ACCEL_SENSITIVITY = 8192.0


def compute_pitch_roll_angles(t, accel_x_data, accel_y_data, accel_z_data,
                              gyro_x_data, gyro_y_data, alpha):
    n = len(gyro_x_data)
    pitch_data = [0.0] * (n + 1)
    roll_data = [0.0] * (n + 1)

    # Use accelerometer data to find 1st angle
    ax = (accel_x_data[0] + accel_x_data[1] + accel_x_data[2] + accel_x_data[3]) / 4.0
    ay = (accel_y_data[0] + accel_y_data[1] + accel_y_data[2] + accel_y_data[3]) / 4.0
    az = (accel_z_data[0] + accel_z_data[1] + accel_z_data[2] + accel_z_data[3]) / 4.0
    pitch_data[0] = math.atan2(ax, az) * 180.0 / math.pi
    roll_data[0] = math.atan2(ay, az) * 180.0 / math.pi

    for i in range(0, n):
        dt = t[i + 1] - t[i]
        pitch_data[i + 1] = pitch_data[i]
        pitch_data[i + 1] -= gyro_y_data[i] * dt

        roll_data[i + 1] = roll_data[i]
        roll_data[i + 1] += gyro_x_data[i] * dt

        pitch_acc = math.atan2(accel_x_data[i], accel_z_data[i]) * 180.0 / math.pi
        roll_acc = math.atan2(accel_y_data[i], accel_z_data[i]) * 180.0 / math.pi

        pitch_data[i + 1] = alpha * pitch_data[i + 1] + (1.0 - alpha) * pitch_acc
        roll_data[i + 1] = alpha * roll_data[i + 1] + (1.0 - alpha) * roll_acc

    return (pitch_data, roll_data)

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

    plt.xlabel('time (s)')
    plt.tight_layout()
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
    accel_x_data = [float(ax) / ACCEL_SENSITIVITY for ax in columns[1]]
    accel_y_data = [float(ay) / ACCEL_SENSITIVITY for ay in columns[2]]
    accel_z_data = [float(az) / ACCEL_SENSITIVITY for az in columns[3]]
    gyro_x_data = [float(gx) / GYRO_SENSITIVITY for gx in columns[4]]
    gyro_y_data = [float(gy) / GYRO_SENSITIVITY for gy in columns[5]]
    gyro_z_data = [float(gz) / GYRO_SENSITIVITY for gz in columns[6]]

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

    plt.xlabel('time (s)')
    plt.ylabel('acceleration (g)')
    plt.tight_layout()
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

    plt.xlabel('time (s)')
    plt.ylabel('angular speed (deg/s)')
    plt.tight_layout()
    plt.savefig('{}/gyro.png'.format(indir))

    # Plot angles
    t.insert(0, t[0])
    (pitch_gyro_data, roll_gyro_data) = \
        compute_pitch_roll_angles(t, accel_x_data, accel_y_data, accel_z_data,
                                  gyro_x_data, gyro_y_data, 1.0)
    (pitch_acc_data, roll_acc_data) = \
        compute_pitch_roll_angles(t, accel_x_data, accel_y_data, accel_z_data,
                                  gyro_x_data, gyro_y_data, 0.0)
    (pitch_filtered_data, roll_filtered_data) = \
        compute_pitch_roll_angles(t, accel_x_data, accel_y_data, accel_z_data,
                                  gyro_x_data, gyro_y_data, 0.98)

    n = len(gyro_x_data)
    heading_gyro_data = [0.0] * (n + 1)
    for i in range(0, n):
        dt = t[i + 1] - t[i]
        heading_gyro_data[i + 1] = heading_gyro_data[i] - gyro_z_data[i] * dt

    plt.figure(3)
    roll_plot = plt.subplot(3, 1, 1)
    roll_plot.set_title('roll')
    roll_plot.plot(t, roll_gyro_data, label='roll (gyro)')
    roll_plot.plot(t, roll_acc_data, label='roll (accel)')
    roll_plot.plot(t, roll_filtered_data, label='roll (filtered)')

    pitch_plot = plt.subplot(3, 1, 2)
    pitch_plot.set_title('pitch')
    pitch_plot.plot(t, pitch_gyro_data, label='pitch (gyro)')
    pitch_plot.plot(t, pitch_acc_data, label='pitch (accel)')
    pitch_plot.plot(t, pitch_filtered_data, label='pitch (filtered)')

    heading_plot = plt.subplot(3, 1, 3)
    heading_plot.set_title('heading')
    heading_plot.plot(t, heading_gyro_data, label='heading (gyro)')

    plt.xlabel('time (s)')
    plt.ylabel('angle (deg)')
    plt.tight_layout()
    plt.savefig('{}/angles.png'.format(indir))

    # Remove gravity vector from acceleration data
    accel_x_data_2 = [0.0] * n
    accel_y_data_2 = [0.0] * n
    accel_z_data_2 = [0.0] * n
    for i in range(0, n):
        gravity_x = 0.0
        gravity_y = 0.0
        gravity_z = -1.0

        # Rotate gravity vector using pitch and roll
        r = roll_filtered_data[i + 1] * math.pi / 180.0
        p = pitch_filtered_data[i + 1] * math.pi / 180.0

        gy = gravity_y
        gz = gravity_z
        gravity_y = gy * math.cos(r) - gz * math.sin(r)
        gravity_z = gy * math.sin(r) + gz * math.cos(r)

        gx = gravity_x
        gz = gravity_z
        gravity_x = gx * math.cos(p) + gz * math.sin(p)
        gravity_z = - gx * math.sin(p) + gz * math.cos(p)

        l = gravity_x * gravity_x + gravity_y * gravity_y + gravity_z * gravity_z
        l = math.sqrt(l)
        gravity_x /= l
        gravity_y /= l
        gravity_z /= l

        accel_x_data_2[i] = accel_x_data[i] + gravity_x
        accel_y_data_2[i] = accel_y_data[i] + gravity_y
        accel_z_data_2[i] = accel_z_data[i] + gravity_z

    # Plot speed
    plt.figure(4)
    speed_data = [0.0] * (n + 1)
    speed_x_data = [0.0] * (n + 1)
    speed_y_data = [0.0] * (n + 1)
    speed_z_data = [0.0] * (n + 1)
    avg_speed_x_data = [0.0] * (n + 1)
    avg_speed_y_data = [0.0] * (n + 1)
    avg_speed_z_data = [0.0] * (n + 1)

    for i in range(0, n):
        dt = t[i + 1] - t[i]
        speed_x_data[i + 1] = speed_x_data[i] + accel_x_data_2[i] * dt
        speed_y_data[i + 1] = speed_y_data[i] + accel_y_data_2[i] * dt
        speed_z_data[i + 1] = speed_z_data[i] + accel_z_data_2[i] * dt

        speed_data[i + 1] = speed_x_data[i + 1] * speed_x_data[i + 1] \
                          + speed_y_data[i + 1] * speed_y_data[i + 1] \
                          + speed_z_data[i + 1] * speed_z_data[i + 1]
        speed_data[i + 1] = math.sqrt(speed_data[i + 1])

    speed_x_plot = plt.subplot(4, 1, 1)
    speed_x_plot.set_title('speed x')
    speed_x_plot.plot(t, speed_x_data, label='speed_x')

    speed_y_plot = plt.subplot(4, 1, 2)
    speed_y_plot.set_title('speed y')
    speed_y_plot.plot(t, speed_y_data, label='speed_y')

    speed_z_plot = plt.subplot(4, 1, 3)
    speed_z_plot.set_title('speed z')
    speed_z_plot.plot(t, speed_z_data, label='speed_z')

    speed_plot = plt.subplot(4, 1, 4)
    speed_plot.set_title('speed')
    speed_plot.plot(t, speed_data, label='speed')

    plt.xlabel('time (s)')
    plt.ylabel('speed (m/s)')
    plt.tight_layout()
    plt.savefig('{}/speed.png'.format(indir))

if len(sys.argv) != 2:
    print("Wrong number of arguments.")
    print(__doc__)
    exit(1)

indir = sys.argv[1]

analyse_radio_data(indir)
analyse_output_data(indir)
analyse_mpu6050_data(indir)
