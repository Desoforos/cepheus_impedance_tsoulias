#include "includes.h"
#include <typeinfo>

std_msgs::Float64 msg_x; //errorx
std_msgs::Float64 msg_y; //errory
std_msgs::Float64 msg_theta; //errortheta
std_msgs::Float64 msg_fextx;
double raw_force_x;
double x = 0, y = 0, theta = 0;
bool shutdown_requested = false;

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

    ros::Subscriber vicon_test_sub = nh.subscribe("/vicon/vicon_test/vicon_test", 1, vicon_testCallback);
    // ros::Subscriber ft_test_sub = nh.subscribe("/bus0/ft_sensor0/ft_sensor_readings/wrench", 1, ft_testCallback);
    ros::Subscriber ft_test_sub = nh.subscribe("/botasys", 1, ft_testCallback);
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

