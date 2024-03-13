//this is a node that reads the desired position from the user, then it parses it wherever it desires
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iostream>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include "define.h"

#include <typeinfo>

/*Kinematics variables*/
double ee_x,ee_y,ee_z;
double q1,q2,q3; //the desired q1,q2

/*From alex thesis*/
double a,b,c,d;
double r0,m0;
double r1,m1,l1;
double r2,m2,l2;
double r3,m3,l3;
double M;
double theta0, thetaee;

/*
void inverseKinematics(){
    double a_cap, b_cap;
    double q1nom, q1denom;
    double q2nom, q2denomElbowUp, q2denomElbowDown;
    a_cap = c*sin(q2);
    b_cap = b + c*cos(q2);
    q2nom = (pow( ee_x - a*cos(theta0) - d*cos(thetaee),2) + (ee_y - a*sin(theta0) - d*sin(thetaee))^2 - b^2 - c^2)/(2*b*c);
    q2denomElbowUp = sqrt(1-q2nom^2);
    q2denomElbowDown = -sqrt(1-q2nom^2);

    q2 = atan2(q2nom,q2denomElbowDown); //GIA ELBOW DOWN LYSH PAPAP

    q1nom = -( a_cap*(ee_x-a*cos(theta0)-d*cos(thetaee))-b_cap*(ee_y-a*sin(theta0)-d*sin(thetaee))/(b^2+c^2+2*b*c*cos(q2)) );
    q1denom = ( a_cap*(ee_y-a*sin(theta0)-d*sin(thetaee))-b_cap*(ee_y-a*cos(theta0)-d*cos(thetaee))/(b^2+c^2+2*b*c*cos(q2)) );
    q1 = atan2(q1nom,q1denom)-theta0;
    q3 = thetaee-theta0-q1-q2;
}
*/

void initialiseParameters(){
    M = m0 + m1 + m2 + m3;
    a = r0*m0/M;
    b = (l1*m0 + (m0+m1)*r1)/M;
    c = (l2*(m0 + m1) + (m0 + m1 + m2)*r2)/M;
    d = r3 + l3*(m0 + m1 + m2)/M;
}

int main(int argc, char **argv) {

    //int ee_x,ee_y,ee_z;

    /* ros init */
    ros::init(argc, argv, "foros_user_node");
    ros::NodeHandle n;

    /* Create publishers */
    ros::Publisher ee_target_pos_pub = n.advertise<geometry_msgs::Pose>("/cepheus/ee_target_pos", 1);

    ROS_INFO("[foros user node]: give me the point (x,y,z) you want the end effector to go \n");
    std::cin >> ee_x >> ee_y >> ee_z;

    geometry_msgs::Pose pose_msg;

    pose_msg.position.x = ee_x;
    pose_msg.position.y = ee_y;
    pose_msg.position.z = ee_z; //dont care about it either
    pose_msg.orientation.x = 0.0;
    pose_msg.orientation.y = 0.0;
    pose_msg.orientation.z = 0.0;
    pose_msg.orientation.w = 0.0;  //we dont care about the orientation for now

    ee_target_pos_pub.publish(pose_msg);

    ros::spinOnce();

    ROS_INFO("[foros user node]: target position published.");

    return 0;
}




