#include <typeinfo>
#include "includes.h"


int main(int argc, char **argv) {

    /* ros init */
    ros::init(argc, argv, "simple_publisher_node");
    ros::NodeHandle nh;

    ros::Publisher LW_torque_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_wrist_effort_controller/command", 100);
    double torque;
    std_msgs::Float64 msg_LW; //left wrist

    std::cout<<"give me torque (Nm)"<<std::endl;
    std::cin>>torque;

    msg_LW.data = torque;

    LW_torque_pub.publish(msg_LW);
    ros::spinOnce();
    msg_LW.data = 0;
    LW_torque_pub.publish(msg_LW);
    ros::spinOnce();


    return 0;
}
