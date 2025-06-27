#include "includes.h"
#include <typeinfo>

std_msgs::Float64 msg_x; //errorx
std_msgs::Float64 msg_y; //errory
std_msgs::Float64 msg_theta; //errortheta
std_msgs::Float64 msg_fextx;
double raw_force_x;
double x = 0, y = 0, theta = 0;
bool shutdown_requested = false;
double imu_acc_x, imu_acc_y, imu_vel_theta;
double q1, q2,q3, q1dot, q2dot, q3dot;
std_msgs::Float64 msg_q1;
std_msgs::Float64 msg_q2;
std_msgs::Float64 msg_q3;
std_msgs::Float64 msg_q1dot;
std_msgs::Float64 msg_q2dot;
std_msgs::Float64 msg_q3dot;

void ft_testCallback(const geometry_msgs::WrenchStamped::ConstPtr&msg){
    raw_force_x = (msg->wrench.force.z);  
}

void vicon_testCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    x = msg->transform.translation.x;
    y = msg->transform.translation.y;
	tf::Quaternion qee( //for angle of ee
		msg->transform.rotation.x,
		msg->transform.rotation.y,
		msg->transform.rotation.z,
		msg->transform.rotation.w);
    tf::Matrix3x3 m_ee(qee);	
    double rollee, pitchee, yawee;
	m_ee.getRPY(rollee, pitchee, yawee);
    theta = yawee;
}

void sigintHandler(int sig) {
    ROS_INFO("Shutdown request received. Performing cleanup tasks...");
    shutdown_requested = true;  // Set flag for graceful shutdown
}
void imu_linear_accCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    imu_acc_x = msg->vector.x;
    imu_acc_y = msg->vector.y;
}

void imu_angular_velCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    imu_vel_theta = msg->vector.z;
}
void lsPosCallback(const std_msgs::Float64::ConstPtr& cmd) {
    // if (firstTimeq1) {
    //     rawq1 = -(cmd->data);
    //     q1 = rawq1 + offsetq1;
    //     firstTimeq1 = false;
    //     }
    // else{
    //     if (abs(-(cmd->data) - rawq1) > qfilter){
    //         //do nothing
    //         }
    //     else{
    //         rawq1 = -(cmd->data); //o encoder tou mou ta diabazei anapoda
    //         q1 = rawq1 +  offsetq1;  
    //         }
    // }

    q1 = -(cmd->data);
   
    // q1 =  -(cmd->data) +  offsetq1; // tha dokimaso to pano
}


void lePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
    // if (firstTimeq2) {
    //     rawq2 = cmd->data;
    //     q2 = rawq2 + offsetq2;
    //     firstTimeq2 = false;
    //     }
    // else{    
    //     if (abs(cmd->data - rawq2) > qfilter){
    //         //do nothing
    //         }
    //     else{
    //         rawq2 = cmd->data; //o encoder tou mou ta diabazei anapoda
    //         q2 = rawq2 +  offsetq2;  
    //         }
    // }
    q2 = cmd->data;
}

void rePosCallback(const std_msgs::Float64::ConstPtr& cmd) {

    q3 = -(cmd->data);

}


void lsVelCallback(const std_msgs::Float64::ConstPtr& cmd) {

    q1dot = -(cmd->data);
}


void leVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	// if (abs(cmd->data - q2dot) > VEL_FILTER)
	// 	return;
	// els
    q2dot = cmd->data;
}


void reVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	// if (abs(cmd->data - q3dot) > VEL_FILTER)
	// 	return;
	// else
    q3dot = -(cmd->data);
}


int main(int argc, char **argv) {

    /* ros init */
    ros::init(argc, argv, "recorder_node");
    ros::NodeHandle nh;

    ros::Time curr_time, t_beg;
    ros::Duration dur_time;
    double secs;
    ros::Rate loop_rate(100); //100Hz
    char command;
    rosbag::Bag bag;
    std::string path = "/home/desoforos/cepheus_impedance_tsoulias/new_rosbags/" ;
    std::string bag_file_name;
    std_msgs::Float64 temp_msg;


    ros::Subscriber vicon_test_sub = nh.subscribe("/vicon/vicon_test/vicon_test", 1, vicon_testCallback);
    // ros::Subscriber ft_test_sub = nh.subscribe("/bus0/ft_sensor0/ft_sensor_readings/wrench", 1, ft_testCallback);
    ros::Subscriber ft_test_sub = nh.subscribe("/botasys", 1, ft_testCallback);
    ros::Subscriber imu_linear_acc_sub = nh.subscribe("/imu/acceleration", 1, imu_linear_accCallback);
    ros::Subscriber imu_angular_vel_sub = nh.subscribe("/imu/angular_velocity", 1, imu_angular_velCallback);
    ros::Subscriber ls_pos_sub = nh.subscribe("read_left_shoulder_position", 1, lsPosCallback);  //gia q1
	ros::Subscriber le_pos_sub = nh.subscribe("read_left_elbow_position", 1, lePosCallback);     //gia q2
  	ros::Subscriber re_pos_sub = nh.subscribe("read_right_elbow_position", 1, rePosCallback);    //gia q3  
 	ros::Subscriber ls_vel_sub = nh.subscribe("read_left_shoulder_velocity", 1, lsVelCallback);  //gia q1dot   
    ros::Subscriber le_vel_sub = nh.subscribe("read_left_elbow_velocity", 1, leVelCallback);       //gia q2dot
   	ros::Subscriber re_vel_sub = nh.subscribe("read_right_elbow_velocity", 1, reVelCallback);    //gia q3dot
    bool record = false;
    int count = 0;
    signal(SIGINT, sigintHandler);

   
    ROS_INFO("[Recorder]: You want to record to a bag? Press Y for yes, anything else for no. \n");
    std::cin>>command;
    if(command == 'Y'){
        record = true;
        ROS_INFO("[Recorder]: Please provide the name of the bag (dont put .bag). \n");
        std::cin >>  bag_file_name;
        bag.open(path + bag_file_name + ".bag", rosbag::bagmode::Write);
    }
    

    while(ros::ok()&& !shutdown_requested){
        ros::spinOnce();
        if(record){
            msg_x.data = x;
            msg_y.data = y;
            msg_theta.data = theta;
            msg_fextx.data = raw_force_x;

            bag.write("/vicon_test/x", ros::Time::now(), msg_x);
            bag.write("/vicon_test/y", ros::Time::now(), msg_y);
            bag.write("/vicon_test/theta", ros::Time::now(), msg_theta);
            bag.write("/ft_test/x", ros::Time::now(), msg_fextx);

            temp_msg.data = imu_acc_x;
            bag.write("/cepheus/imu_acc_x", ros::Time::now(), temp_msg);    

            temp_msg.data = imu_acc_y;
            bag.write("/cepheus/imu_acc_y", ros::Time::now(), temp_msg);

            temp_msg.data = imu_vel_theta;
            bag.write("/cepheus/imu_vel_theta", ros::Time::now(), temp_msg);     
            
            msg_q1.data = q1;
            msg_q2.data = q2;
            msg_q3.data = q3;   

            msg_q1dot.data = q1dot;
            msg_q2dot.data = q2dot;
            msg_q3dot.data = q3dot;


            bag.write("/cepheus/q1", ros::Time::now(), msg_q1);
            bag.write("/cepheus/q2", ros::Time::now(), msg_q2);
            bag.write("/cepheus/q3", ros::Time::now(), msg_q3);
    

            bag.write("/cepheus/q1dot", ros::Time::now(), msg_q1dot);
            bag.write("/cepheus/q2dot", ros::Time::now(), msg_q2dot);
            bag.write("/cepheus/q3dot", ros::Time::now(), msg_q3dot);
        }
        if(count%100 == 0){
            ROS_INFO("[Recorder]: fextx is: %f N. \n",raw_force_x);
            ROS_INFO("[Recorder]: vicon test x is: %f N. \n",x);
            ROS_INFO("[Recorder]: vicon test y is: %f N. \n",y);
            ROS_INFO("[Recorder]: vicon test theta is is: %f N. \n",theta);
        }
        count++;
        loop_rate.sleep();
    }

    if(shutdown_requested && record){
        bag.close();
    } 
    return 0;

}

