#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Wrench.h>
#include <control_msgs/JointControllerState.h>

class ImpedanceControl {
public:
    ImpedanceControl() : nh("~") {
        // Initialize ROS parameters
        nh.param<std::string>("joint1_controller", joint1_controller, "joint1_position_controller");
        nh.param<std::string>("joint2_controller", joint2_controller, "joint2_position_controller");

        // Subscribe to joint states
        joint_state_sub = nh.subscribe("/joint_states", 1, &ImpedanceControl::jointStateCallback, this);

        // Subscribe to external force
        force_sub = nh.subscribe("/external_force", 1, &ImpedanceControl::forceCallback, this);

        // Advertise torque commands
        torque_pub1 = nh.advertise<std_msgs::Float64>("/" + joint1_controller + "/command", 1);
        torque_pub2 = nh.advertise<std_msgs::Float64>("/" + joint2_controller + "/command", 1);

        // Initialize gains and impedance parameters
        stiffness = 100.0;  // Adjust as needed
        damping = 5.0;     // Adjust as needed

        // Initialize joint angles
        joint1_angle = 0.0;
        joint2_angle = 0.0;

        // Set loop rate
        loop_rate = ros::Rate(100.0);  // Adjust as needed
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // Update joint angles
        joint1_angle = msg->position[0];
        joint2_angle = msg->position[1];
    }

    void forceCallback(const geometry_msgs::Wrench::ConstPtr& msg) {
        // Impedance control calculation
        double force_error = target_force - msg->force.z;
        double torque_cmd = stiffness * (target_position - joint1_angle) - damping * msg->torque.z;

        // Publish torque commands
        std_msgs::Float64 torque_msg;
        torque_msg.data = torque_cmd;
        torque_pub1.publish(torque_msg);

        // For the second joint, you can implement a similar control law
        // ...

        // Rate limit to control loop
        loop_rate.sleep();
    }

    void run() {
        while (ros::ok()) {
            ros::spinOnce();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber joint_state_sub;
    ros::Subscriber force_sub;
    ros::Publisher torque_pub1;
    ros::Publisher torque_pub2;
    ros::Rate loop_rate;

    std::string joint1_controller;
    std::string joint2_controller;

    double joint1_angle;
    double joint2_angle;

    double target_position = 0.0;  // Desired end-effector position
    double target_force = 10.0;    // Desired force

    double stiffness;
    double damping;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "impedance_control_node");
    ImpedanceControl impedance_control;
    impedance_control.run();
    return 0;
}
