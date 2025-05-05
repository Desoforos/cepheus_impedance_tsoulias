#ifndef ROBOT_CALLBACKS_H
#define ROBOT_CALLBACKS_H
#include "includes.h"
#include "robot_variables.h"

/////////////// CALLBACK FUNCTIONS DEFINITION START////////////////////////




double moving_average(double new_value, std::deque<double>& window, int size, double& running_sum) {
    if (window.size() == size) {
        running_sum -= window.front();
        window.pop_front();
    }
    window.push_back(new_value);
    running_sum += new_value;
    return running_sum / window.size();
}


void ee_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    if(!eecheck){
        eecheck = true;
    }
    if(!firstTime){
        xE_prev = ee_x;
        yE_prev = ee_y;
        thetaE_prev = thetach;
    }
    ee_x = msg->transform.translation.x;
    ee_y = msg->transform.translation.y;
	tf::Quaternion qee( //for angle of ee
		msg->transform.rotation.x,
		msg->transform.rotation.y,
		msg->transform.rotation.z,
		msg->transform.rotation.w);
    tf::Matrix3x3 m_ee(qee);	
    double rollee, pitchee, yawee;
	m_ee.getRPY(rollee, pitchee, yawee);
	// thetach = yawee; //angle of chaser(ee)
    if(firstTime){
        thetach = yawee;
    }
    else{
        if(abs(yawee - thetach) < 8*M_PI/180){
            thetach = yawee; //angle of chaser(ee)
        }
    }
}


void target_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    if(!targetcheck){
        targetcheck = true;
    }
    if(!firstTime){
        xt_prev = xt;
        yt_prev = yt;
        thetat_prev = thetat;
    }
    xt = msg->transform.translation.x;
	yt = msg->transform.translation.y;
	tf::Quaternion qt( //for angle of ee
		msg->transform.rotation.x,
		msg->transform.rotation.y,
		msg->transform.rotation.z,
		msg->transform.rotation.w);
    tf::Matrix3x3 m_t(qt);	
    double rollt, pitcht, yawt;
	m_t.getRPY(rollt, pitcht, yawt);
    if(firstTime){
        thetat = yawt;
    }
    else{
        if(abs(yawt - thetat) < 8*M_PI/180){
            thetat = yawt; //angle of chaser(ee)

        }
    }
}

void base_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    if(!basecheck){
        basecheck = true;
    }
    if(!firstTime){
        xc0_prev = xc0;
        yc0_prev = yc0;
        theta0_prev = theta0;
    }
	xc0 = msg->transform.translation.x;
	yc0 = msg->transform.translation.y;
	tf::Quaternion qc0( 
		msg->transform.rotation.x,
		msg->transform.rotation.y,
		msg->transform.rotation.z,
		msg->transform.rotation.w);
    tf::Matrix3x3 m_c0(qc0);	
    double rollc0, pitchc0, yawc0;
	m_c0.getRPY(rollc0, pitchc0, yawc0);
    if(firstTime){
        theta0 = yawc0;
    }
    else{
        if(abs(yawc0 - theta0) < 8*M_PI/180){
            theta0 = yawc0; 
        }
    }
}
	




void lsPosCallback(const std_msgs::Float64::ConstPtr& cmd) {
    if(offsetsdone){
        q1 = moving_average(-(cmd->data)+ offsetq1, q1_window, 10,sumq1);
    }
    else{
        q1 = -(cmd->data) + offsetq1;
    }
}


void lePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
    if(offsetsdone){
        q2 = moving_average(cmd->data + offsetq2, q2_window, 10,sumq2);
    }
    else{
        q2 = cmd->data + offsetq2;
    }
}

void rePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
    if(offsetsdone){
        q3 = moving_average(-(cmd->data)+ offsetq3, q3_window, 10,sumq3);
    }
    else{
        q3 = -(cmd->data) + offsetq3;
    }
}


void lsVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
    if(offsetsdone){
        q1dot = moving_average(-(cmd->data), q1dot_window, 10,sumq1dot);
    }
    else{
        q1dot = -(cmd->data);
    }
}


void leVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
    if(offsetsdone){
        q2dot = moving_average(cmd->data, q2dot_window, 10,sumq2dot);
    }
    else{
        q2dot = cmd->data;
    }
}


void reVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
    if(offsetsdone){
        q3dot = moving_average(-(cmd->data), q3dot_window, 10,sumq3dot);
    }
    else{
        q3dot = -(cmd->data);
    }
}




void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr&msg){
    raw_force_x = (msg->wrench.force.z);  //etsi einai mapped apo to bota  filtered.
    raw_force_x = moving_average(raw_force_x, force_window, 10, forcesum);
    force_x = 0; 
	if(abs(force_x)<1){
		incontact = false;
	}
	else{
		incontact = true;
	}
}

void arduinoCallbacktest(const std_msgs::String::ConstPtr &msg){
	if(msg->data == "nothing"){
		if(gripperListenedSoft){
			softFinished = true;
		}
		if(gripperListenedHard){
			hardFinished = true;
		}
	}
	if(msg->data == "softgrip"){
		gripperListenedSoft = true;
	}
	if(msg->data == "hardgrip"){
		gripperListenedHard = true;
	}
}

/////////////// CALLBACK FUNCTIONS DEFINITION END////////////////////////
#endif