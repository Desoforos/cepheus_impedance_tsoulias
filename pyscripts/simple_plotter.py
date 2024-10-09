#!/usr/bin/env python3

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import sys

from std_msgs.msg import Float64

# Bag file path
path = "/home/desoforos/cepheus_impedance_tsoulias/rosbags/"
bag_file = path + sys.argv[1] + ".bag"



# Open bag file
# bag = rosbag.Bag(bag_file)

#!/usr/bin/env python3



# Make sure the user provides the bag file as a command-line argument
if len(sys.argv) < 2:
    print("Usage: python3 plot_torque_vs_xeethetadotdot.py <bag_file>")
    sys.exit()

# Get the bag file from the command line arguments

# Initialize lists to store the data
time_stamps = []
torques = []
xeethetadotdot = []

# Open the rosbag
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/cepheus/left_wrist_torque', '/cepheus/xeethetadotdot']):
        time_stamps.append(t.to_sec())  # Use the same timestamps for both topics
        if topic == '/cepheus/left_wrist_torque':
            torques.append(msg.data)
            # print(f"Read torque: {msg.data} at time {t.to_sec()}")
        elif topic == '/cepheus/xeethetadotdot':
            xeethetadotdot.append(msg.data)
            # print(f"Read xeethetadotdot: {msg.data} at time {t.to_sec()}")

# Ensure both lists have the same number of elements
min_length = min(len(torques), len(xeethetadotdot))
torques = torques[:min_length]
xeethetadotdot = xeethetadotdot[:min_length]
time_stamps = time_stamps[:min_length]

# Debugging output for the lengths of the lists
# print(f"Number of timestamps: {len(time_stamps)}")
# print(f"Number of torques: {len(torques)}")
# print(f"Number of xeethetadotdots: {len(xeethetadotdot)}")

# Plot the data if there is data to plot
if len(time_stamps) > 0:
    plt.figure()

    # Plot left wrist torque
    plt.plot(time_stamps, torques, label='/left_wrist_torque', color='r')

    # Plot xeethetadot
    plt.plot(time_stamps, xeethetadotdot, label='/xeethetadotdot', color='b')

    # Add labels, title, and legend
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('Left Wrist Torque and Xee Theta Dot')
    plt.legend()

    # Show the plot
    plt.grid()
    plt.show()
else:
    print("No data to plot.")
