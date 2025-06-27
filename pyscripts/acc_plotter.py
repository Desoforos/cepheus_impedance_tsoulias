#!/usr/bin/env python3

import sys
import os
import rosbag
import matplotlib.pyplot as plt

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_acc_from_bag.py <your_bag_file.bag>")
        sys.exit(1)

    input_name = sys.argv[1]
    base_path = "/home/desoforos/cepheus_impedance_tsoulias/new_rosbags/"
    bag_filename = os.path.join(base_path, input_name)
    bag_filename = bag_filename + ".bag"

    if not os.path.exists(bag_filename):
        print(f"Error: {bag_filename} does not exist.")
        sys.exit(1)

    acc_data = {}
    acc_active = [False] * 6

    topics = []
    for i in range(1, 7):
        for axis in ['x', 'y', 'z']:
            topic = f"/acc{i}_{axis}"
            topics.append(topic)
            acc_data[topic] = {'time': [], 'data': []}

    print(f"Reading from bag: {bag_filename}")
    bag = rosbag.Bag(bag_filename)
    for topic, msg, t in bag.read_messages(topics=topics):
        acc_data[topic]['time'].append(t.to_sec())
        acc_data[topic]['data'].append(msg.data)
    bag.close()

    for i in range(6):
        has_data = any(
            len(acc_data[f"/acc{i+1}_{axis}"]['data']) > 0
            for axis in ['x', 'y', 'z']
        )
        acc_active[i] = has_data

    num_active = sum(acc_active)
    if num_active == 0:
        print("No accelerometer data found in bag.")
        sys.exit(0)

    print(f"Plotting {num_active} active accelerometer(s)...")
    fig, axs = plt.subplots(num_active, 1, figsize=(12, 3 * num_active), sharex=True)
    if num_active == 1:
        axs = [axs]

    plot_idx = 0
    for i in range(6):
        if not acc_active[i]:
            continue

        ax = axs[plot_idx]
        for axis in ['x', 'y', 'z']:
            topic = f"/acc{i+1}_{axis}"
            if acc_data[topic]['time']:
                t0 = acc_data[topic]['time'][0]
                times = [t - t0 for t in acc_data[topic]['time']]
                ax.plot(times, acc_data[topic]['data'], label=axis)

        ax.set_ylabel(f"acc{i+1} (m/s²)")
        ax.legend(loc='upper right')
        ax.grid(True)
        plot_idx += 1

    axs[-1].set_xlabel("Time (s)")
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.suptitle("Accelerometer Data from Bag", fontsize=16)
    plt.show()

if __name__ == "__main__":
    main()
