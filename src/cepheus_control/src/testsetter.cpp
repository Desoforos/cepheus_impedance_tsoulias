#include "includes.h"
#include <controller_manager_msgs/SwitchController.h>

int main(int argc, char **argv) {

    /* ros init */
    ros::init(argc, argv, "testsetter_node");
    ros::NodeHandle nh;
    ros::Time curr_time, t_beg;
    // ros::Duration dur_time; //duration of movement
    double dur_time;

    /* Create publishers */
    ros::Publisher LS_position_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_shoulder_position_controller/command", 1);
    ros::Publisher LE_position_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_elbow_position_controller/command", 1);
    ros::Publisher LW_position_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_wrist_position_controller/command", 1);

    /* messages to publish */
    std_msgs::Float64 msg_LS;
    std_msgs::Float64 msg_LE;
    std_msgs::Float64 msg_LW;

    double q1,q2,q3;

    ROS_INFO("[testsetter]: Warning! Do not proceed before running the Gazebo. \n");

    std::cout <<"Give me initial q1(shoulder) (deg):" <<std::endl;
    std::cin >> q1;
    std::cout <<"Give me initial q2(elbow) (deg):" <<std::endl;
    std::cin >> q2;
    std::cout <<"Give me initial q3(wrist) (deg):" <<std::endl;
    std::cin >> q3;

    q1 = q1*M_PI/180; //from deg to rad
    q2 = q2*M_PI/180; //from deg to rad
    q3 = q3*M_PI/180; //from deg to rad

    msg_LS.data = q1;
    msg_LE.data = q2;
    msg_LW.data = q3;

    LS_position_pub.publish(msg_LS);
    LE_position_pub.publish(msg_LE);
    LW_position_pub.publish(msg_LW);

    ros::spinOnce();

    ROS_INFO("[testsetter]: Initial pose of Cepheus established. \n");
    
    ros::Duration(5.0).sleep();

     // Switch controllers
    ros::ServiceClient switch_client = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    // Switch to effort controllers
    controller_manager_msgs::SwitchController switch_srv;
    switch_srv.request.start_controllers = {"left_shoulder_effort_controller", "left_elbow_effort_controller", "left_wrist_effort_controller"};
    switch_srv.request.stop_controllers = {"left_shoulder_position_controller", "left_elbow_position_controller", "left_wrist_position_controller"};
    switch_srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
    
    

    if (switch_client.call(switch_srv)) {
        ROS_INFO("Switched to effort controllers");
    } else {
        ROS_ERROR("Failed to switch to effort controllers");
        return 1;
    }

    ros::shutdown();

    return 0;
}