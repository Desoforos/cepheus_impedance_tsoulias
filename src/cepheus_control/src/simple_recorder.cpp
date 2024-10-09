#include "includes.h"

double xeethetadot =0;
double torque =0;
bool shutdown_requested = false;


void sigintHandler(int sig) {
    ROS_INFO("Shutdown request received. Performing cleanup tasks...");
    shutdown_requested = true;  // Set flag for graceful shutdown
}

void gazeboposCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){
    for(int i=0; i<msg->name.size(); i++){
		if(msg->name[i] == "cepheus::final_link"){
			xeethetadot= msg->twist[i].angular.z;
		}
    }
}

void torqueCallback(const std_msgs::Float64::ConstPtr& msg){
    torque = msg->data;
}



int main(int argc, char **argv) {

    /* ros init */
    ros::init(argc, argv, "simple_recorder_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);

    ros::Subscriber gazebo_pos_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",100,gazeboposCallback);
    // ros::Subscriber joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/cepheus/joint_states",100,jointStatesCallback);
    ros::Subscriber LW_torque_sub = nh.subscribe<std_msgs::Float64>("/cepheus/left_wrist_effort_controller/command",100,torqueCallback);

    rosbag::Bag bag;
    std::string path = "/home/desoforos/cepheus_impedance_tsoulias/rosbags/" ;
    std::string bag_file_name;
    std_msgs::Float64 msg_xeethetadot;
    std_msgs::Float64 msg_torque;




    ROS_INFO("[new_foros_simcontroller]: Please provide the name of the bag (dont put .bag). \n");
    std::cin >>  bag_file_name;
    bag.open(path + bag_file_name + ".bag", rosbag::bagmode::Write);

    

    ROS_INFO("Starting recording of ee_thetadot, ee_thetadotdot, torque of ee.");
    while(ros::ok() && !shutdown_requested){
        ros::spinOnce();
        msg_xeethetadot.data = xeethetadot;
        msg_torque.data = torque;


        bag.write("/cepheus/left_wrist_torque", ros::Time::now(), msg_torque);
        bag.write("/cepheus/xeethetadot", ros::Time::now(), msg_xeethetadot);

        if(shutdown_requested){
            bag.close();
            }
        }

    return 0;
}
