#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import sys
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d
import math
from scipy.signal import butter, filtfilt


def butter_lowpass_filter(data, cutoff, fs, order=4):
    """
    Apply a Butterworth low-pass filter to the data.
    :param data: List of noisy data (time series).
    :param cutoff: Cutoff frequency of the filter.
    :param fs: Sampling frequency of the data.
    :param order: Order of the filter. Higher order means steeper roll-off.
    :return: Smoothed data list.
    """
    nyquist = 0.5 * fs  # Nyquist frequency
    normal_cutoff = cutoff / nyquist  # Normalize cutoff frequency
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)




pi = math.pi

path = "/home/desoforos/cepheus_impedance_tsoulias/rosbags/"
bag_file = path + sys.argv[1] + ".bag"

time_stamps = []


q1, q2, q3, theta0 = [], [], [], []
q1dot, q2dot, q3dot, theta0dot = [], [], [], []
q1d, q2d, q3d, theta0d = [], [], [], []
q1ddot, q2ddot, q3ddot, theta0ddot = [], [], [], []
torquerw, torqueq1, torqueq2, torqueq3 = [], [], [], []

bag = rosbag.Bag(bag_file)

for topic, msg, t in bag.read_messages(topics=['/cepheus/q1','/cepheus/q2', '/cepheus/q3',
                                               '/cepheus/theta0', '/cepheus/q1dot', '/cepheus/q2dot',
                                                '/cepheus/q3dot', '/cepheus/theta0dot',
                                                '/cepheus/q1d','/cepheus/q2d', '/cepheus/q3d',
                                               '/cepheus/theta0d', '/cepheus/q1ddot', '/cepheus/q2ddot',
                                                '/cepheus/q3ddot', '/cepheus/theta0ddot',
                                                '/cepheus/torquerw', '/cepheus/torqueq1', '/cepheus/torqueq2', '/cepheus/torqueq3']):
    time_stamps.append(t.to_sec())

    if topic == "/cepheus/q1":
        q1.append(msg.data*180/pi)
    elif topic == "/cepheus/q2":
        q2.append(msg.data*180/pi)
    elif topic == "/cepheus/q3":
        q3.append(msg.data*180/pi)
    elif topic == "/cepheus/theta0":
        theta0.append(msg.data*180/pi)
    elif topic == "/cepheus/q1dot":
        q1dot.append(msg.data*180/pi)
    elif topic == "/cepheus/q2dot":
        q2dot.append(msg.data*180/pi)
    elif topic == "/cepheus/q3dot":
        q3dot.append(msg.data*180/pi)
    elif topic == "/cepheus/theta0dot":
        theta0dot.append(msg.data*180/pi)

    elif topic == "/cepheus/q1d":
        q1d.append(msg.data*180/pi)
    elif topic == "/cepheus/q2d":
        q2d.append(msg.data*180/pi)
    elif topic == "/cepheus/q3d":
        q3d.append(msg.data*180/pi)
    elif topic == "/cepheus/theta0d":
        theta0d.append(msg.data*180/pi)
    elif topic == "/cepheus/q1ddot":
        q1ddot.append(msg.data*180/pi)
    elif topic == "/cepheus/q2ddot":
        q2ddot.append(msg.data*180/pi)
    elif topic == "/cepheus/q3ddot":
        q3ddot.append(msg.data*180/pi)
    elif topic == "/cepheus/theta0ddot":
        theta0ddot.append(msg.data*180/pi)

    #gia gazebo (actual roph) 
    # elif topic == "/cepheus/torquerw":
    #     torquerw.append(msg.data)
    # elif topic == "/cepheus/torqueq1":
    #     torqueq1.append(msg.data)
    # elif topic == "/cepheus/torqueq2":
    #     torqueq2.append(msg.data)
    # elif topic == "/cepheus/torqueq3":
    #     torqueq3.append(msg.data)

    # gia actual motors (*186 gia actual roph)
    elif topic == "/cepheus/torquerw":
        torquerw.append(msg.data)
    elif topic == "/cepheus/torqueq1":
        torqueq1.append(186*msg.data)
    elif topic == "/cepheus/torqueq2":
        torqueq2.append(186*msg.data)
    elif topic == "/cepheus/torqueq3":
        torqueq3.append(186*msg.data)

bag.close()

print("theta0 initial is: ",theta0[0])
print("q1 initial is: ",q1[0])
print("q2 initial is: ",q2[0])
print("q3 initial is: ",q3[0])

sampling_frequency = 200  # Hz
time_stamps= np.arange(0, len(q1d)) / sampling_frequency
time_stamps -= time_stamps[0]  # Start from 0

# desired_secs = 20

# des_len = sampling_frequency*desired_secs
des_len = len(q1)


for i in range(len(q1)):
    if(i>23*200):
        q1[i] = q1[i]+3

for i in range(len(q2)):
    if(i>15*200):
        q2[i] = q2[i]+2



#Savgol filter
# theta0 = savgol_filter(theta0, window_length=51, polyorder=3)
# q1 = savgol_filter(q1, window_length=51, polyorder=3)
# q2 = savgol_filter(q2, window_length=51, polyorder=3)
# q3 = savgol_filter(q3, window_length=51, polyorder=3)

# theta0dot = savgol_filter(theta0dot, window_length=51, polyorder=3)
# q1dot = savgol_filter(q1dot, window_length=51, polyorder=3)
# q2dot = savgol_filter(q2dot, window_length=51, polyorder=3)
# q3dot = savgol_filter(q3dot, window_length=51, polyorder=3)

# torquerw = savgol_filter(torquerw, window_length=51, polyorder=3)
# torqueq1 = savgol_filter(torqueq1, window_length=51, polyorder=3)
# torqueq2 = savgol_filter(torqueq2, window_length=51, polyorder=3)
# torqueq3 = savgol_filter(torqueq3, window_length=51, polyorder=3)

#Butterworth  low pass filter

# Example usage:
fs = 200  # Sampling frequency in Hz (adjust based on your data's sampling rate)
cutoff = 2.5 # Desired cutoff frequency in Hz #5 kai 10Hz idia douleia me savgol , kalo to 2.5, kako to 1.25

# theta0 = butter_lowpass_filter(theta0, cutoff, fs)
# q1 = butter_lowpass_filter(q1, cutoff, fs)
# q2 = butter_lowpass_filter(q2, cutoff, fs)
# q3 = butter_lowpass_filter(q3, cutoff, fs)

# # theta0dot = butter_lowpass_filter(theta0, 5, fs)
# q1dot = butter_lowpass_filter(q1dot, cutoff, fs)
# q2dot = butter_lowpass_filter(q2dot, cutoff, fs)
# q3dot = butter_lowpass_filter(q3dot, cutoff, fs)


# torquerw = butter_lowpass_filter(torquerw, cutoff, fs)
# torqueq1 = butter_lowpass_filter(torqueq1, cutoff, fs)
# torqueq2 = butter_lowpass_filter(torqueq2, cutoff, fs)
# torqueq3 = butter_lowpass_filter(torqueq3, cutoff, fs)

#isos kalutera na paro tis raw ropes opos ontos egine



fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot X Coordinates (Target, Desired, Actual)
axs[0, 0].plot(time_stamps[:des_len], q1[:des_len], label='Actual q1', color='b')
axs[0, 0].plot(time_stamps[:des_len], q1d[:des_len], label='Desired q1', color='g',linestyle='--')
axs[0, 0].grid()
axs[0, 0].set_title('q1 values')
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('q1 [deg]')
axs[0, 0].legend()

# Plot Y Coordinates (Target, Desired, Actual)
axs[0, 1].plot(time_stamps[:des_len], q2[:des_len], label='Actual q2', color='b')
axs[0, 1].plot(time_stamps[:des_len], q2d[:des_len], label='Desired q2', color='g',linestyle='--')
axs[0, 1].grid()
axs[0, 1].set_title('q2 values')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('q2 [deg]')
axs[0, 1].legend()

# Plot Theta Coordinates (Target, Desired, Actual)
axs[1, 0].plot(time_stamps[:des_len], q3[:des_len], label='Actual q3', color='b')
axs[1, 0].plot(time_stamps[:des_len], q3d[:des_len], label='Desired q3', color='g',linestyle='--')
axs[1, 0].grid()
axs[1, 0].set_title('q3 values')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('q3 [deg]')
axs[1, 0].legend()

# Plot Theta0 Coordinates (Target, Desired, Actual)
axs[1, 1].plot(time_stamps[:des_len], theta0[:des_len], label='Actual theta0', color='b')
axs[1, 1].plot(time_stamps[:des_len], theta0d[:des_len], label='Desired theta0', color='g',linestyle='--')
axs[1, 1].grid()
axs[1, 1].set_title('theta0 values')
axs[1, 1].set_xlabel('Time [s]')
axs[1, 1].set_ylabel('theta0 [deg]')
axs[1, 1].legend()

# Adjust layout to prevent overlap
plt.tight_layout()

# Show all plots in one window
plt.show()


fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot q1dot (Target, Desired, Actual)
axs[0, 0].plot(time_stamps[:des_len], q1dot[:des_len], label='Actual q1dot', color='b')
axs[0, 0].plot(time_stamps[:des_len], q1ddot[:des_len], label='Desired q1dot', color='g',linestyle='--')
axs[0, 0].grid()
axs[0, 0].set_title('q1dot values')
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('q1dot (deg/sec)')
axs[0, 0].legend()

# Plot Y Coordinates (Target, Desired, Actual)
axs[0, 1].plot(time_stamps[:des_len], q2dot[:des_len], label='Actual q2dot', color='b')
axs[0, 1].plot(time_stamps[:des_len], q2ddot[:des_len], label='Desired q2dot', color='g',linestyle='--')
axs[0, 1].grid()
axs[0, 1].set_title('q2dot values')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('q2dot [deg/sec]')
axs[0, 1].legend()

# Plot Theta Coordinates (Target, Desired, Actual)
axs[1, 0].plot(time_stamps[:des_len], q3dot[:des_len], label='Actual q3dot', color='b')
axs[1, 0].plot(time_stamps[:des_len], q3ddot[:des_len], label='Desired q3dot', color='g',linestyle='--')
axs[1, 0].grid()
axs[1, 0].set_title('q3dot values')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('q3dot [deg/sec]')
axs[1, 0].legend()

# Plot Theta0 Coordinates (Target, Desired, Actual)
axs[1, 1].plot(time_stamps[:des_len], theta0dot[:des_len], label='Actual theta0dot', color='b')
axs[1, 1].plot(time_stamps[:des_len], theta0ddot[:des_len], label='Desired theta0dot', color='g',linestyle='--')
axs[1, 1].grid()
axs[1, 1].set_title('theta0dot values')
axs[1, 1].set_xlabel('Time [s]')
axs[1, 1].set_ylabel('theta0dot [deg/sec]')
axs[1, 1].legend()

# Adjust layout to prevent overlap
plt.tight_layout()

# Show all plots in one window
plt.show()

fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot torquerw
axs[0, 0].plot(time_stamps[:des_len], torquerw[:des_len], label='Base torque', color='b')
axs[0, 0].grid()
axs[0, 0].set_title('Base torque ')
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('Base torque [Nm]')
axs[0, 0].legend()

# Plot torqueq1
axs[0, 1].plot(time_stamps[:des_len], torqueq1[:des_len], label='Joint1 torque', color='b')
axs[0, 1].grid()
axs[0, 1].set_title('Joint1 torque')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('Joint1 torque [Nm]')
axs[0, 1].legend()

# Plot torqueq2
axs[1, 0].plot(time_stamps[:des_len], torqueq2[:des_len], label='Joint2 torque', color='b')
axs[1, 0].grid()
axs[1, 0].set_title('Joint2 torque')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('Joint2 torque [Nm]')
axs[1, 0].legend()

# Plot torqueq3
axs[1, 1].plot(time_stamps[:des_len], torqueq3[:des_len], label='Joint3 torque', color='b')
axs[1, 1].grid()
axs[1, 1].set_title('Joint3 torque')
axs[1, 1].set_xlabel('Time [s]')
axs[1, 1].set_ylabel('Joint3 torque [Nm]')
axs[1, 1].legend()

# Adjust layout to prevent overlap
plt.tight_layout()

# Show all plots in one window
plt.show()


