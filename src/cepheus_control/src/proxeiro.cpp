#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "testsetter_node");
    ros::NodeHandle nh;

    // Publisher for joint states
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    // Set initial joint positions
    sensor_msgs::JointState joint_state;
    joint_state.name = {"joint1", "joint2", "joint3"};
    joint_state.position = {M_PI / 4, M_PI / 4, M_PI / 4};  // Replace with desired initial positions

    ros::Rate rate(10);
    for (int i = 0; i < 10; ++i) {
        joint_state.header.stamp = ros::Time::now();
        joint_state_pub.publish(joint_state);
        rate.sleep();
    }

    // Create a service client for the /controller_manager/switch_controller service
    ros::ServiceClient switch_client = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

    // Ensure the service is available
    switch_client.waitForExistence();

    // Prepare the service request to switch to position controllers
    controller_manager_msgs::SwitchController switch_srv;
    switch_srv.request.start_controllers = {"joint1_position_controller", "joint2_position_controller", "joint3_position_controller"};
    switch_srv.request.stop_controllers = {"joint1_effort_controller", "joint2_effort_controller", "joint3_effort_controller"};
    switch_srv.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;

    // Call the service to switch to position controllers
    if (switch_client.call(switch_srv)) {
        if (switch_srv.response.ok) {
            ROS_INFO("Switched to position controllers");
        } else {
            ROS_ERROR("Failed to switch to position controllers: %s", switch_srv.response.ok ? "Success" : "Failure");
        }
    } else {
        ROS_ERROR("Service call to switch to position controllers failed");
        return 1;
    }

    // Wait a bit to ensure the positions are set
    ros::Duration(5.0).sleep();

    // Prepare the service request to switch to effort controllers
    switch_srv.request.start_controllers = {"joint1_effort_controller", "joint2_effort_controller", "joint3_effort_controller"};
    switch_srv.request.stop_controllers = {"joint1_position_controller", "joint2_position_controller", "joint3_position_controller"};

    // Call the service to switch to effort controllers
    if (switch_client.call(switch_srv)) {
        if (switch_srv.response.ok) {
            ROS_INFO("Switched to effort controllers");
        } else {
            ROS_ERROR("Failed to switch to effort controllers: %s", switch_srv.response.ok ? "Success" : "Failure");
        }
    } else {
        ROS_ERROR("Service call to switch to effort controllers failed");
        return 1;
    }

    ros::shutdown();
    return 0;
}

