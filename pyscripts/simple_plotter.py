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
    print("Usage: python3 plot_torque_vs_accgazebo.py <bag_file>")
    sys.exit()

# Get the bag file from the command line arguments

# Initialize lists to store the data
time_stamps = []
torques = []
accgazebo = []
accjoint = []
accimu = []

# Open the rosbag
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/cepheus/left_wrist_torque', '/cepheus/accgazebo','/cepheus/accjoint' , '/cepheus/accimu' ]):
        time_stamps.append(t.to_sec())  # Use the same timestamps for both topics
        if topic == '/cepheus/left_wrist_torque':
            torques.append(msg.data)
            # print(f"Read torque: {msg.data} at time {t.to_sec()}")
        elif topic == '/cepheus/accgazebo':
            accgazebo.append(msg.data)
            # print(f"Read accgazebo: {msg.data} at time {t.to_sec()}")
        elif topic == '/cepheus/accjoint':
            accjoint.append(msg.data)
            # print(f"Read accjoint: {msg.data} at time {t.to_sec()}")
        elif topic == '/cepheus/accimu':
            accimu.append(msg.data)
            # print(f"Read accimu: {msg.data} at time {t.to_sec()}")

# Ensure both lists have the same number of elements
# min_length = min(len(torques), len(accgazebo))
# torques = torques[:min_length]
# accgazebo = accgazebo[:min_length]
# accjoint = accjoint[:min_length]
# accimu = accimu[:min_length]

# time_stamps = time_stamps[:min_length]
sampling_frequency = 100  # Hz
time_stamps= np.arange(0, len(accgazebo)) / sampling_frequency
# time_stamps -= time_stamps[0]  # Start from 0

# Debugging output for the lengths of the lists
# print(f"Number of timestamps: {len(time_stamps)}")
# print(f"Number of torques: {len(torques)}")
# print(f"Number of accgazebos: {len(accgazebo)}")

# Plot the data if there is data to plot
if len(time_stamps) > 0:
    plt.figure()


    # plt.plot(time_stamps, torques, label='/left_wrist_torque', color='r')
    plt.plot(time_stamps, accgazebo, label='/acc_gazebo', color='b')
    plt.plot(time_stamps, accjoint, label='/acc_joint', color='g')
    plt.plot(time_stamps, accimu, label='/acc_imu', color='cyan')

    # Add labels, title, and legend
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('Left Wrist Torque and Angular Accelerations')
    plt.legend()

    # Show the plot
    plt.grid()
    plt.show()
else:
    print("No data to plot.")
