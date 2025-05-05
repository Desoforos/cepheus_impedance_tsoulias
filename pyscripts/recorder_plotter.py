#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import sys
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d
import math
pi = math.pi
from scipy.signal import butter, filtfilt
import scipy.signal as signal
import numpy as np
import scipy.signal as signal
import scipy.ndimage as ndimage
import scipy.io
import os

# Bag file path
path = "/home/desoforos/cepheus_impedance_tsoulias/new_rosbags/"
path2 = "/home/desoforos/cepheus_impedance_tsoulias/gia_rek/"
bag_file = path + sys.argv[1] + ".bag"

time_stamps = []
x, y, theta = [], [], []
fextx = []

data_dict = {
    "fext": [],
    "time": [],
    "x": [],
    "y": [],
    "theta": []
}

# Open bag file
bag = rosbag.Bag(bag_file)

# Read the bag file
for topic, msg, t in bag.read_messages(topics=['/vicon_test/x','/vicon_test/y','/vicon_test/theta','/ft_test/x']):
    if topic == "/vicon_test/x":
        x.append(msg.data)
    elif topic == "/vicon_test/y":
        y.append(msg.data)
    elif topic == "/vicon_test/theta":
        theta.append(msg.data)
    elif topic == "/ft_test/x":
        fextx.append(msg.data)

bag.close()



# Ask the user for the filename
filename = input("Enter the filename to save (without .mat extension): ")
filename = filename.strip() + ".mat"  # Ensure the file has a .mat extension


sampling_frequency = 100  # Hz

# cutoff_index = 12 * 100
# fextx = fextx[cutoff_index:]
time_stamps= np.arange(0, len(x)) / sampling_frequency
time_stamps -= time_stamps[0]  # Start from 0

data_dict["time"] = time_stamps
data_dict["fext"] = np.array(fextx)
data_dict["x"] = np.array(x)
data_dict["y"] = np.array(y)
data_dict["theta"] = np.array(theta)

file_path = os.path.join(path2, filename)

scipy.io.savemat(file_path, data_dict)

print(f"Data saved to {filename}")

# desired_secs = 60
# des_len = sampling_frequency*desired_secs
des_len = len(fextx)

# External Force (X-axis)
plt.figure()
plt.plot(time_stamps[:des_len], fextx[:des_len], label='Fext X rokubimi mini', color='k')
plt.grid()
plt.title('External Force (X-axis)')
plt.xlabel('Time [s]')
plt.ylabel('Force [N]')
plt.legend()

# Show all plots
plt.show()

fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Plot X values (Target, Desired, Actual)
axs[0, 0].plot(time_stamps[:des_len], x[:des_len], label='X', color='r')
axs[0, 0].grid()
axs[0, 0].set_title('X values')
axs[0, 0].set_xlabel('Time [s]')
axs[0, 0].set_ylabel('X Position [m]')
axs[0, 0].legend()

# Plot Y values (Target, Desired, Actual)
axs[0, 1].plot(time_stamps[:des_len], y[:des_len], label='Y', color='r')
axs[0, 1].grid()
axs[0, 1].set_title('Y values')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('Y Position [m]')
axs[0, 1].legend()

# Plot Theta values (Target, Desired, Actual)
axs[1, 0].plot(time_stamps[:des_len], theta[:des_len], label='Theta', color='r')
axs[1, 0].grid()
axs[1, 0].set_title('Theta values')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('Theta [deg]')
axs[1, 0].legend()

# Adjust layout to prevent overlap
plt.tight_layout()

# Show all plots in one window
plt.show()

