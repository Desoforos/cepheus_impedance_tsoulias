#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include "includes.h"

bool switchControllers(ros::NodeHandle& nh, 
                       const std::vector<std::string>& stop_controllers, 
                       const std::vector<std::string>& start_controllers) {
    ros::ServiceClient switch_client = nh.serviceClient<controller_manager_msgs::SwitchController>("/cepheus/controller_manager/switch_controller");

    controller_manager_msgs::SwitchController switch_srv;
    switch_srv.request.stop_controllers = stop_controllers;
    switch_srv.request.start_controllers = start_controllers;
    switch_srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

    if (switch_client.call(switch_srv)) {
        return switch_srv.response.ok;
    } else {
        ROS_ERROR("Failed to call service switch_controller");
        return false;
    }
}

bool loadController(ros::NodeHandle& nh, const std::string& controller_name) {
    ros::ServiceClient load_client = nh.serviceClient<controller_manager_msgs::LoadController>("/cepheus/controller_manager/load_controller");

    controller_manager_msgs::LoadController load_srv;
    load_srv.request.name = controller_name;

    if (load_client.call(load_srv)) {
        return load_srv.response.ok;
    } else {
        ROS_ERROR("Failed to call service load_controller");
        return false;
    }
}

bool unloadController(ros::NodeHandle& nh, const std::string& controller_name) {
    ros::ServiceClient unload_client = nh.serviceClient<controller_manager_msgs::UnloadController>("/cepheus/controller_manager/unload_controller");

    controller_manager_msgs::UnloadController unload_srv;
    unload_srv.request.name = controller_name;

    if (unload_client.call(unload_srv)) {
        return unload_srv.response.ok;
    } else {
        ROS_ERROR("Failed to call service unload_controller");
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "testsetter_node");
    ros::NodeHandle nh;

    std::vector<std::string> controllersA = {"left_shoulder_position_controller", "left_elbow_position_controller", "left_wrist_position_controller"};
    std::vector<std::string> controllersB = {"left_shoulder_effort_controller", "left_elbow_effort_controller", "left_wrist_effort_controller"};

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
    
    ros::Duration(3.0).sleep();
    //////////////////////////CONTROLLER SWITCHING////////////////////////////////

    // Step 1: Stop controllers A1, A2, A3
    if (!switchControllers(nh, controllersA, {})) {
        ROS_ERROR("Failed to stop controllers A1, A2, A3");
        return 1;
    }

    // Step 2: Unload controllers A1, A2, A3
    for (const auto& controller : controllersA) {
        if (!unloadController(nh, controller)) {
            ROS_ERROR("Failed to unload controller %s", controller.c_str());
            return 1;
        }
    }

    // Step 3: Load controllers B1, B2, B3
    for (const auto& controller : controllersB) {
        if (!loadController(nh, controller)) {
            ROS_ERROR("Failed to load controller %s", controller.c_str());
            return 1;
        }
    }

    // Step 4: Start controllers B1, B2, B3
    if (!switchControllers(nh, {}, controllersB)) {
        ROS_ERROR("Failed to start controllers B1, B2, B3");
        return 1;
    }

    ROS_INFO("Successfully switched from position controllers to effort controllers.");

    ros::spinOnce();
    return 0;
}
