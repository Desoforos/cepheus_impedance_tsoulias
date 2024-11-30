rostopic pub /cmd_torque std_msgs/Float64 "0.0001" --once

rostopic pub /set_left_shoulder_effort std_msgs/Float64 "0.0001" --once

rostopic pub /set_left_elbow_effort std_msgs/Float64 "0.0001" --once

rostopic pub /set_right_elbow_effort std_msgs/Float64 "0.0001" --once
