#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import sys

# Bag file path
path = "/home/desoforos/cepheus_impedance_tsoulias/rosbags/"
bag_file = path + sys.argv[1] + ".bag"

# Initialize lists for storing data
time_stamps = []
xt_x, xd_x, xee_x = [], [], []
xt_y, xd_y, xee_y = [], [], []
xt_theta, xd_theta, xee_theta = [], [], []
fext_x = []

# Open bag file
bag = rosbag.Bag(bag_file)

# Read the bag file
for topic, msg, t in bag.read_messages(topics=['/cepheus/xt_x', '/cepheus/xd_x', '/cepheus/xee_x',
                                               '/cepheus/xt_y', '/cepheus/xd_y', '/cepheus/xee_y',
                                               '/cepheus/xt_theta', '/cepheus/xd_theta', '/cepheus/xee_theta',
                                               '/cepheus/ft_sensor_topic']):

    # Time
    time_stamps.append(t.to_sec())

    # X values
    if topic == "/cepheus/xt_x":
        xt_x.append(msg.data)
    elif topic == "/cepheus/xd_x":
        xd_x.append(msg.data)
    elif topic == "/cepheus/xee_x":
        xee_x.append(msg.data)

    # Y values
    elif topic == "/cepheus/xt_y":
        xt_y.append(msg.data)
    elif topic == "/cepheus/xd_y":
        xd_y.append(msg.data)
    elif topic == "/cepheus/xee_y":
        xee_y.append(msg.data)

    # Theta values
    elif topic == "/cepheus/xt_theta":
        xt_theta.append(msg.data)
    elif topic == "/cepheus/xd_theta":
        xd_theta.append(msg.data)
    elif topic == "/cepheus/xee_theta":
        xee_theta.append(msg.data)

    # External force (x-axis)
    elif topic == "/cepheus/ft_sensor_topic":
        fext_x.append(msg.data)

bag.close()

# Convert time stamps to relative time
time_stamps = np.array(time_stamps)
time_stamps -= time_stamps[0]  # Start from 0

# Create plots

# X Coordinates (Target, Desired, Actual)
plt.figure()
plt.plot(time_stamps[:len(xt_x)], xt_x, label='Target X', color='r')
plt.plot(time_stamps[:len(xd_x)], xd_x, label='Desired X', color='g')
plt.plot(time_stamps[:len(xee_x)], xee_x, label='Actual X', color='b')
plt.title('X Coordinates')
plt.xlabel('Time [s]')
plt.ylabel('X Position [m]')
plt.legend()

# Y Coordinates (Target, Desired, Actual)
plt.figure()
plt.plot(time_stamps[:len(xt_y)], xt_y, label='Target Y', color='r')
plt.plot(time_stamps[:len(xd_y)], xd_y, label='Desired Y', color='g')
plt.plot(time_stamps[:len(xee_y)], xee_y, label='Actual Y', color='b')
plt.title('Y Coordinates')
plt.xlabel('Time [s]')
plt.ylabel('Y Position [m]')
plt.legend()

# Theta Coordinates (Target, Desired, Actual)
plt.figure()
plt.plot(time_stamps[:len(xt_theta)], xt_theta, label='Target Theta', color='r')
plt.plot(time_stamps[:len(xd_theta)], xd_theta, label='Desired Theta', color='g')
plt.plot(time_stamps[:len(xee_theta)], xee_theta, label='Actual Theta', color='b')
plt.title('Theta Coordinates')
plt.xlabel('Time [s]')
plt.ylabel('Theta [rad]')
plt.legend()

# External Force (X-axis)
plt.figure()
plt.plot(time_stamps[:len(fext_x)], fext_x, label='Fext X', color='k')
plt.title('External Force (X-axis)')
plt.xlabel('Time [s]')
plt.ylabel('Force [N]')
plt.legend()

# Show all plots
plt.show()
