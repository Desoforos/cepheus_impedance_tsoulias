#ifndef ROBOT_CALLBACKS_H
#define ROBOT_CALLBACKS_H
#include "includes.h"
#include "robot_variables.h"


void gazeboposCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);


void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);


void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr&msg);










#endif