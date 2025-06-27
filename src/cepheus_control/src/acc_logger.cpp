#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Float32.h>
#include <signal.h>
#include <string>
#include <vector>
#include <iostream>

rosbag::Bag bag;
bool running = true;

void signalHandler(int signum) {
    ROS_INFO("Interrupt signal (%d) received. Closing bag.", signum);
    running = false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "acc_recorder");
    ros::NodeHandle nh;

    if (argc < 2) {
        ROS_ERROR("Usage: rosrun <your_package> acc_recorder <bag_name.bag>");
        return 1;
    }

    std::string filename = argv[1];
    std::string path = "/home/desoforos/cepheus_impedance_tsoulias/new_rosbags/";
    std::string full_path = path + filename + ".bag";

    bag.open(full_path, rosbag::bagmode::Write);
    signal(SIGINT, signalHandler);  // graceful shutdown

    std::vector<std::string> topic_names;
    for (int i = 1; i <= 6; ++i) {
        for (const std::string& axis : {"x", "y", "z"}) {
            topic_names.push_back("/acc" + std::to_string(i) + "_" + axis);
        }
    }

    std::vector<ros::Subscriber> subscribers;

    for (const auto& topic : topic_names) {
        subscribers.push_back(nh.subscribe<std_msgs::Float32>(topic, 100,
            [topic](const std_msgs::Float32::ConstPtr& msg) {
                bag.write(topic, ros::Time::now(), *msg);
            }
        ));
    }

    ROS_INFO("Recording accelerometer topics to: %s", full_path.c_str());

    ros::Rate rate(100);
    while (ros::ok() && running) {
        ros::spinOnce();
        rate.sleep();
    }

    bag.close();
    ROS_INFO("Bag file closed.");

    return 0;
}
