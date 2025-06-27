#!/bin/bash

# Check if a name was provided
if [ -z "$1" ]; then
  echo "Usage: $0 <bag_filename>"
  exit 1
fi

BAG_NAME=$1

# Replace with your actual rosrun command
rosrun rosbag record -O /home/desoforos/cepheus_impedance_tsoulias/gia_rek/"$BAG_NAME".bag /read_left_shoulder_position
                                             /read_left_elbow_position
