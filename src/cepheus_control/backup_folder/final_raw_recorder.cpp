#include "includes.h"
#include <typeinfo>




bool shutdown_requested = false;
double q1, q2,q3, q1dot, q2dot, q3dot;
std_msgs::Float64 msg_q1;
std_msgs::Float64 msg_q2;
std_msgs::Float64 msg_q3;
std_msgs::Float64 msg_q1dot;
std_msgs::Float64 msg_q2dot;
std_msgs::Float64 msg_q3dot;
std_msgs::Float64 marker_x;
std_msgs::Float64 marker_y;
std_msgs::Float64 marker_z;
geometry_msgs::WrenchStamped ft_msg;
geometry_msgs::Vector3Stamped linear_acc_msg;
geometry_msgs::Vector3Stamped ang_vel_msg;


void sigintHandler(int sig) {
    ROS_INFO("Shutdown request received. Performing cleanup tasks...");
    shutdown_requested = true;  // Set flag for graceful shutdown
}

void ft_testCallback(const geometry_msgs::WrenchStamped::ConstPtr&msg){
    ft_msg = *msg;  
}

void vicon_testCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    marker_x.data = msg->transform.translation.x;
    marker_y.data = msg->transform.translation.y;
    marker_z.data = msg->transform.translation.z;
}


void imu_linear_accCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    linear_acc_msg = *msg;
}

void imu_angular_velCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    ang_vel_msg = *msg;
}
void lsPosCallback(const std_msgs::Float64::ConstPtr& cmd) {
    q1 = -(cmd->data);
    msg_q1.data = q1;
}


void lePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
    q2 = cmd->data;
    msg_q2.data = q2;
}

void rePosCallback(const std_msgs::Float64::ConstPtr& cmd) {

    q3 = -(cmd->data);
    msg_q3.data = q3;

}


void lsVelCallback(const std_msgs::Float64::ConstPtr& cmd) {

    q1dot = -(cmd->data);
    msg_q1dot.data = q1dot;
}


void leVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
    q2dot = cmd->data;
    msg_q2dot.data = q2dot;
}


void reVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
    q3dot = -(cmd->data);
    msg_q3dot.data = q3dot;
}




int main(int argc, char **argv) {

    /* ros init */
    ros::init(argc, argv, "final_raw_recorder_node");
    ros::NodeHandle nh;

    ros::Time curr_time, t_beg;
    ros::Duration dur_time;
    double secs;
    ros::Rate loop_rate(100); //100Hz
    char command;
    signal(SIGINT, sigintHandler);

    rosbag::Bag bag;
    std::string path = "/home/desoforos/cepheus_impedance_tsoulias/gia_rek/" ;
    std::string bag_file_name;
    std_msgs::Float64 temp_msg;


    ros::Subscriber vicon_test_sub = nh.subscribe("/vicon/markers", 1, vicon_testCallback); //vicon marker xyz
    ros::Subscriber ft_test_sub = nh.subscribe("/bus0/ft_sensor0/ft_sensor_readings/wrench", 1, ft_testCallback); //bota f/t
    ros::Subscriber imu_linear_acc_sub = nh.subscribe("/imu/acceleration", 1, imu_linear_accCallback);// imu acc
    ros::Subscriber imu_angular_vel_sub = nh.subscribe("/imu/angular_velocity", 1, imu_angular_velCallback); //imu ang vel
    ros::Subscriber ls_pos_sub = nh.subscribe("read_left_shoulder_position", 1, lsPosCallback);  //gia q1
	ros::Subscriber le_pos_sub = nh.subscribe("read_left_elbow_position", 1, lePosCallback);     //gia q2
  	ros::Subscriber re_pos_sub = nh.subscribe("read_right_elbow_position", 1, rePosCallback);    //gia q3  
 	ros::Subscriber ls_vel_sub = nh.subscribe("read_left_shoulder_velocity", 1, lsVelCallback);  //gia q1dot   
    ros::Subscriber le_vel_sub = nh.subscribe("read_left_elbow_velocity", 1, leVelCallback);       //gia q2dot
   	ros::Subscriber re_vel_sub = nh.subscribe("read_right_elbow_velocity", 1, reVelCallback);    //gia q3dot


    ROS_INFO("[Recorder]: Please provide the name of the bag (dont put .bag). \n");
    std::cin >>  bag_file_name;
    bag.open(path + bag_file_name + ".bag", rosbag::bagmode::Write);

    while(ros::ok()&& !shutdown_requested){
        ros::spinOnce();
        bag.write("/vicon/marker_x", ros::Time::now(), marker_x);
        bag.write("/vicon/marker_y", ros::Time::now(), marker_y);
        bag.write("/vicon/marker_z", ros::Time::now(), marker_z);
        bag.write("/bus0/ft_sensor0/ft_sensor_readings/wrench", ros::Time::now(), ft_msg);
        bag.write("/imu/acceleration", ros::Time::now(), linear_acc_msg);
        bag.write("/imu/angular_velocity", ros::Time::now(), ang_vel_msg);
        bag.write("read_left_shoulder_position", ros::Time::now(), msg_q1);
        bag.write("read_left_elbow_position", ros::Time::now(), msg_q2);
        bag.write("read_right_elbow_position", ros::Time::now(), msg_q3);
        bag.write("read_left_shoulder_velocity", ros::Time::now(), msg_q1dot);
        bag.write("read_left_elbow_velocity", ros::Time::now(), msg_q2dot);
        bag.write("read_right_elbow_velocity", ros::Time::now(), msg_q3dot);

        loop_rate.sleep(); //100Hz
    }

    if(shutdown_requested){
        bag.close();
    } 

    return 0;
}