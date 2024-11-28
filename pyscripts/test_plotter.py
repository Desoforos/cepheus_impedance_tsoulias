#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import numpy as np
import sys
from scipy.signal import savgol_filter



def plot_rw_torque(bag_file_path, topic_name):
    # Initialize lists for time and torque values
    times = []
    torques = []

    # Open the ROS bag
    try:
        with rosbag.Bag(bag_file_path, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=[topic_name]):
                times.append(t.to_sec())  # Convert time to seconds
                torques.append(msg.data)  # Read torque value
    except Exception as e:
        print(f"Error reading bag file: {e}")
        return

    # Convert time to relative time (start at 0)
    if times:
        start_time = times[0]
        times = [time - start_time for time in times]
    
    # Plot the data
    plt.figure(figsize=(10, 6))
    plt.plot(times, torques, label='Left Shoulder Torque', color='blue', linestyle='-', linewidth=1.5)
    plt.xlabel('Time (s)', fontsize=14)
    plt.ylabel('Torque (Nm)', fontsize=14)
    plt.grid(True)
    plt.legend(fontsize=12)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Change these paths as needed
    path = "/home/desoforos/cepheus_impedance_tsoulias/pyscripts/"
    bag_file_path = path + sys.argv[1] + ".bag"
     
    topic_name = "/cepheus/left_shoulder_effort_controller/command"  # Topic to read

    plot_rw_torque(bag_file_path, topic_name)
