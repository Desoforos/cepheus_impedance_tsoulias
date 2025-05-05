#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import sys
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d
import math
from scipy.signal import butter, filtfilt

pi = math.pi

path = "/home/desoforos/cepheus_impedance_tsoulias/rosbags/"
bag_file = path + sys.argv[1] + ".bag"

time_stamps = []

force_x =[] 
raw_force_x = []

bag = rosbag.Bag(bag_file)

for topic, msg, t in bag.read_messages(topics=['/cepheus/raw_force_x',
                                                '/cepheus/force_x']):
    time_stamps.append(t.to_sec())

    if topic == "/cepheus/raw_force_x":
        raw_force_x.append(msg.data)
    elif topic == "/cepheus/force_x":
        force_x.append(msg.data)

bag.close()

sampling_frequency = 200  # Hz
time_stamps= np.arange(0, len(force_x)) / sampling_frequency
time_stamps -= time_stamps[0]  # Start from 0

plt.figure()
plt.plot(time_stamps[:len(force_x)], force_x, label='Fext_x', color='k')
plt.plot(time_stamps[:len(raw_force_x)], raw_force_x, label='Raw Fext_x')
plt.grid()
plt.title('External Force (X-axis)')
plt.xlabel('Time [s]')
plt.ylabel('Force [N]')
plt.legend()

# Show all plots
plt.show()
