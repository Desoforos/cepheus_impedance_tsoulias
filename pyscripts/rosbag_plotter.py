#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import sys
from scipy.signal import savgol_filter

# Bag file path
path = "/home/desoforos/cepheus_impedance_tsoulias/rosbags/"
bag_file = path + sys.argv[1] + ".bag"

# Initialize lists for storing data
time_stamps = []
xt_x, xd_x, xee_x = [], [], []
xt_y, xd_y, xee_y = [], [], []
xt_theta, xd_theta, xee_theta = [], [], []
xt_theta0, xd_theta0, xee_theta0 = [], [], []
fext_x = []

xt_x_dot, xd_x_dot, xee_x_dot = [], [], []
xt_y_dot, xd_y_dot, xee_y_dot = [], [], []
xt_theta_dot, xd_theta_dot, xee_theta_dot = [], [], []
xt_theta0_dot, xd_theta0_dot, xee_theta0_dot = [], [], []

torquerw, torqueq1, torqueq2, torqueq3 = [], [], [], []



# Open bag file
bag = rosbag.Bag(bag_file)

# Read the bag file
for topic, msg, t in bag.read_messages(topics=['/cepheus/xt_x', '/cepheus/xd_x', '/cepheus/xee_x',
                                               '/cepheus/xt_y', '/cepheus/xd_y', '/cepheus/xee_y',
                                               '/cepheus/xt_theta', '/cepheus/xd_theta', '/cepheus/xee_theta',
                                               '/cepheus/xt_theta0', '/cepheus/xd_theta0', '/cepheus/xee_theta0',
                                               '/cepheus/ft_sensor_topic',
                                               '/cepheus/xt_x_dot', '/cepheus/xd_x_dot', '/cepheus/xee_x_dot',
                                               '/cepheus/xt_y_dot', '/cepheus/xd_y_dot', '/cepheus/xee_y_dot',
                                               '/cepheus/xt_theta_dot', '/cepheus/xd_theta_dot', '/cepheus/xee_theta_dot',
                                               '/cepheus/xt_theta0_dot', '/cepheus/xd_theta0_dot', '/cepheus/xee_theta0_dot',
                                               '/cepheus/torquerw', '/cepheus/torqueq1', '/cepheus/torqueq2', '/cepheus/torqueq3']):

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
    
    # Theta0 values
    elif topic == "/cepheus/xt_theta0":
        xt_theta0.append(msg.data)
    elif topic == "/cepheus/xd_theta0":
        xd_theta0.append(msg.data)
    elif topic == "/cepheus/xee_theta0":
        xee_theta0.append(msg.data)


    # Xdot values
    if topic == "/cepheus/xt_x_dot":
        xt_x_dot.append(msg.data)
    elif topic == "/cepheus/xd_x_dot":
        xd_x_dot.append(msg.data)
    elif topic == "/cepheus/xee_x_dot":
        xee_x_dot.append(msg.data)

    # Ydot values
    elif topic == "/cepheus/xt_y_dot":
        xt_y_dot.append(msg.data)
    elif topic == "/cepheus/xd_y_dot":
        xd_y_dot.append(msg.data)
    elif topic == "/cepheus/xee_y_dot":
        xee_y_dot.append(msg.data)

    # Thetadot values
    elif topic == "/cepheus/xt_theta_dot":
        xt_theta_dot.append(msg.data)
    elif topic == "/cepheus/xd_theta_dot":
        xd_theta_dot.append(msg.data)
    elif topic == "/cepheus/xee_theta_dot":
        xee_theta_dot.append(msg.data)
    
    #Theta0dot values
    elif topic == "/cepheus/xt_theta0_dot":
        xt_theta0_dot.append(msg.data)
    elif topic == "/cepheus/xd_theta0_dot":
        xd_theta0_dot.append(msg.data)
    elif topic == "/cepheus/xee_theta0_dot":
        xee_theta0_dot.append(msg.data)    
    
    #Fextx values
    elif topic == "/cepheus/ft_sensor_topic":
        fext_x.append(-msg.data)
    
    #Torque values
    elif topic == "/cepheus/torquerw":
        torquerw.append(msg.data)
    elif topic == "/cepheus/torqueq1":
        torqueq1.append(msg.data)
    elif topic == "/cepheus/torqueq2":
        torqueq2.append(msg.data)
    elif topic == "/cepheus/torqueq3":
        torqueq3.append(msg.data)





bag.close()

# Convert time stamps to relative time
# time_stamps = np.array(time_stamps)

#diorthosh
sampling_frequency = 200  # Hz
time_stamps= np.arange(0, len(xt_x)) / sampling_frequency
time_stamps -= time_stamps[0]  # Start from 0

desired_secs = 30

des_len = sampling_frequency*desired_secs



#smoothen the plots:
xee_x = savgol_filter(xee_x, window_length=51, polyorder=3)
xee_y = savgol_filter(xee_y, window_length=51, polyorder=3)
xee_theta = savgol_filter(xee_theta, window_length=51, polyorder=3)
xee_theta0 = savgol_filter(xee_theta0, window_length=51, polyorder=3)
# # fext_x = savgol_filter(fext_x, window_length=51, polyorder=3)  #mou xalaei ta arnhtika
xee_x_dot = savgol_filter(xee_x_dot, window_length=51, polyorder=3)
xee_y_dot = savgol_filter(xee_y_dot, window_length=51, polyorder=3)
xee_theta_dot = savgol_filter(xee_theta_dot, window_length=51, polyorder=3)
xee_theta0_dot = savgol_filter(xee_theta0_dot, window_length=51, polyorder=3)
torquerw = savgol_filter(torquerw, window_length=51, polyorder=3)
torqueq1 = savgol_filter(torqueq1, window_length=51, polyorder=3)
torqueq2 = savgol_filter(torqueq2, window_length=51, polyorder=3)
torqueq3 = savgol_filter(torqueq3, window_length=51, polyorder=3)


# # Create plots

# # X Coordinates (Target, Desired, Actual)
# plt.figure()
# plt.plot(time_stamps[:len(xt_x)], xt_x, label='Target X', color='r')
# plt.plot(time_stamps[:len(xd_x)], xd_x, label='Desired X', color='g')
# plt.plot(time_stamps[:len(xee_x)], xee_x, label='Actual X', color='b')
# plt.grid()
# plt.title('X Coordinates')
# plt.xlabel('Time [s]')
# plt.ylabel('X Position [m]')
# plt.legend()

# # Y Coordinates (Target, Desired, Actual)
# plt.figure()
# plt.plot(time_stamps[:len(xt_y)], xt_y, label='Target Y', color='r')
# plt.plot(time_stamps[:len(xd_y)], xd_y, label='Desired Y', color='g')
# plt.plot(time_stamps[:len(xee_y)], xee_y, label='Actual Y', color='b')
# plt.grid()
# plt.title('Y Coordinates')
# plt.xlabel('Time [s]')
# plt.ylabel('Y Position [m]')
# plt.legend()

# # Theta Coordinates (Target, Desired, Actual)
# plt.figure()
# plt.plot(time_stamps[:len(xt_theta)], xt_theta, label='Target Theta', color='r')
# plt.plot(time_stamps[:len(xd_theta)], xd_theta, label='Desired Theta', color='g')
# plt.plot(time_stamps[:len(xee_theta)], xee_theta, label='Actual Theta', color='b')
# plt.grid()
# plt.title('Theta Coordinates')
# plt.xlabel('Time [s]')
# plt.ylabel('Theta [rad]')
# plt.legend()

# # External Force (X-axis)
# plt.figure()
# plt.plot(time_stamps[:len(fext_x)], fext_x, label='Fext X', color='k')
# plt.grid()
# plt.title('External Force (X-axis)')
# plt.xlabel('Time [s]')
# plt.ylabel('Force [N]')
# plt.legend()

# # Show all plots
# plt.show()

# Create a figure and a 2x2 grid of subplots
fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot X Coordinates (Target, Desired, Actual)
axs[0, 0].plot(time_stamps[:len(xt_x)], xt_x, label='Target X', color='r')
axs[0, 0].plot(time_stamps[:len(xd_x)], xd_x, label='Desired X', color='g')
axs[0, 0].plot(time_stamps[:len(xee_x)], xee_x, label='Actual X', color='b')
axs[0, 0].grid()
axs[0, 0].set_title('X Coordinates')
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('X Position [m]')
axs[0, 0].legend()

# Plot Y Coordinates (Target, Desired, Actual)
axs[0, 1].plot(time_stamps[:len(xt_y)], xt_y, label='Target Y', color='r')
axs[0, 1].plot(time_stamps[:len(xd_y)], xd_y, label='Desired Y', color='g')
axs[0, 1].plot(time_stamps[:len(xee_y)], xee_y, label='Actual Y', color='b')
axs[0, 1].grid()
axs[0, 1].set_title('Y Coordinates')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('Y Position [m]')
axs[0, 1].legend()

# Plot Theta Coordinates (Target, Desired, Actual)
axs[1, 0].plot(time_stamps[:len(xt_theta)], xt_theta, label='Target Theta', color='r')
axs[1, 0].plot(time_stamps[:len(xd_theta)], xd_theta, label='Desired Theta', color='g')
axs[1, 0].plot(time_stamps[:len(xee_theta)], xee_theta, label='Actual Theta', color='b')
axs[1, 0].grid()
axs[1, 0].set_title('Theta Coordinates')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('Theta [rad]')
axs[1, 0].legend()

# Plot Theta0 Coordinates (Target, Desired, Actual)
axs[1, 1].plot(time_stamps[:len(xt_theta0)], xt_theta0, label='Target Theta0', color='r')
axs[1, 1].plot(time_stamps[:len(xd_theta0)], xd_theta0, label='Desired Theta0', color='g')
axs[1, 1].plot(time_stamps[:len(xee_theta0)], xee_theta0, label='Actual Theta0', color='b')
axs[1, 1].grid()
axs[1, 1].set_title('Theta0 Coordinates')
axs[1, 1].set_xlabel('Time [s]')
axs[1, 1].set_ylabel('Theta0 [rad]')
axs[1, 1].legend()


# # Plot External Force (X-axis)
# axs[1, 1].plot(time_stamps[:len(fext_x)], fext_x, label='Fext X', color='k')
# axs[1, 1].grid()
# axs[1, 1].set_title('External Force (X-axis)')
# axs[1, 1].set_xlabel('Time [s]')
# axs[1, 1].set_ylabel('Force [N]')
# axs[1, 1].legend()

# Adjust layout to prevent overlap
plt.tight_layout()

# Show all plots in one window
plt.show()


fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot Xdot Coordinates (Target, Desired, Actual)
axs[0, 0].plot(time_stamps[:len(xt_x_dot)], xt_x_dot, label='Target Xdot', color='r')
axs[0, 0].plot(time_stamps[:len(xd_x_dot)], xd_x_dot, label='Desired Xdot', color='g')
axs[0, 0].plot(time_stamps[:len(xee_x_dot)], xee_x_dot, label='Actual Xdot', color='b')
axs[0, 0].grid()
axs[0, 0].set_title('Xdot ')
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('X Velcotiy [m/sec]')
axs[0, 0].legend()

# Plot Ydot Coordinates (Target, Desired, Actual)
axs[0, 1].plot(time_stamps[:len(xt_y_dot)], xt_y_dot, label='Target Ydot', color='r')
axs[0, 1].plot(time_stamps[:len(xd_y_dot)], xd_y_dot, label='Desired Ydot', color='g')
axs[0, 1].plot(time_stamps[:len(xee_y_dot)], xee_y_dot, label='Actual Ydot', color='b')
axs[0, 1].grid()
axs[0, 1].set_title('Ydot')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('Y Velocity [m/sec]')
axs[0, 1].legend()

# Plot Thetadot Coordinates (Target, Desired, Actual)
axs[1, 0].plot(time_stamps[:len(xt_theta_dot)], xt_theta_dot, label='Target Thetadot', color='r')
axs[1, 0].plot(time_stamps[:len(xd_theta_dot)], xd_theta_dot, label='Desired Thetadot', color='g')
axs[1, 0].plot(time_stamps[:len(xee_theta_dot)], xee_theta_dot, label='Actual Thetadot', color='b')
axs[1, 0].grid()
axs[1, 0].set_title('Thetadot')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('Thetadot [rad/sec]')
axs[1, 0].legend()

# Plot Thetadot Coordinates (Target, Desired, Actual)
axs[1, 1].plot(time_stamps[:len(xt_theta0_dot)], xt_theta0_dot, label='Target Theta0dot', color='r')
axs[1, 1].plot(time_stamps[:len(xd_theta0_dot)], xd_theta0_dot, label='Desired Theta0dot', color='g')
axs[1, 1].plot(time_stamps[:len(xee_theta0_dot)], xee_theta0_dot, label='Actual Theta0dot', color='b')
axs[1, 1].grid()
axs[1, 1].set_title('Theta0dot')
axs[1, 1].set_xlabel('Time [s]')
axs[1, 1].set_ylabel('Theta0dot [rad/sec]')
axs[1, 1].legend()


# # Plot External Force (X-axis)
# axs[1, 1].plot(time_stamps[:len(fext_x)], fext_x, label='Fext X', color='k')
# axs[1, 1].grid()
# axs[1, 1].set_title('External Force (X-axis)')
# axs[1, 1].set_xlabel('Time [s]')
# axs[1, 1].set_ylabel('Force [N]')
# axs[1, 1].legend()

# Adjust layout to prevent overlap
plt.tight_layout()

# Show all plots in one window
plt.show()





fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot torquerw
axs[0, 0].plot(time_stamps[:len(torquerw)], torquerw, label='Base torque', color='b')
axs[0, 0].grid()
axs[0, 0].set_title('Base torque ')
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('Base torque [Nm]')
axs[0, 0].legend()

# Plot torqueq1
axs[0, 1].plot(time_stamps[:len(torqueq1)], torqueq1, label='Joint1 torque', color='b')
axs[0, 1].grid()
axs[0, 1].set_title('Joint1 torque')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('Joint1 torque [Nm]')
axs[0, 1].legend()

# Plot torqueq2
axs[1, 0].plot(time_stamps[:len(torqueq2)], torqueq2, label='Joint2 torque', color='b')
axs[1, 0].grid()
axs[1, 0].set_title('Joint2 torque')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('Joint2 torque [Nm]')
axs[1, 0].legend()

# Plot torqueq3
axs[1, 1].plot(time_stamps[:len(torqueq3)], torqueq3, label='Joint3 torque', color='b')
axs[1, 1].grid()
axs[1, 1].set_title('Joint3 torque')
axs[1, 1].set_xlabel('Time [s]')
axs[1, 1].set_ylabel('Joint3 torque [Nm]')
axs[1, 1].legend()

# Adjust layout to prevent overlap
plt.tight_layout()

# Show all plots in one window
plt.show()


# External Force (X-axis)
plt.figure()
plt.plot(time_stamps[:len(fext_x)], fext_x, label='Fext X', color='k')
plt.grid()
plt.title('External Force (X-axis)')
plt.xlabel('Time [s]')
plt.ylabel('Force [N]')
plt.legend()

# Show all plots
plt.show()




# time_before_contact = time_stamps[:des_len]
# torque_before_contact = torquerw[:des_len]

# plt.figure(figsize=(10, 6))

# plt.plot(time_before_contact, torque_before_contact, label="Before Contact (Detailed)")
# plt.grid()

# plt.xlabel("Time (s)")
# plt.ylabel("Torque (Nm)")
# plt.title("Torque of Joint Over Time")
# plt.legend()
# plt.show()