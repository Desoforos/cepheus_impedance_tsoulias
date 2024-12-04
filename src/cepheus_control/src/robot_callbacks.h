#ifndef ROBOT_CALLBACKS_H
#define ROBOT_CALLBACKS_H
#include "includes.h"
#include "robot_variables.h"

/////////////// CALLBACK FUNCTIONS DEFINITION START////////////////////////
// geometry_msgs/TransformStamped

std::deque<double> q1_window;  // Stores the last N values
std::deque<double> q2_window;  // Stores the last N values
std::deque<double> q3_window;  // Stores the last N values

std::deque<double> q1dot_window;  // Stores the last N values
std::deque<double> q2dot_window;  // Stores the last N values
std::deque<double> q3dot_window;  // Stores the last N values


const int window_size = 20;     // Size of the sliding window

double sumq1 = 0, sumq2 = 0, sumq3 =0;
double sumq1dot = 0, sumq2dot = 0, sumq3dot =  0;

double moving_average(double new_value, std::deque<double>& window, int size, double& sum) {
    sum += new_value;
    window.push_back(new_value);               // Add the new value
    if (window.size() > size){
        sum -= window[0];
        window.pop_front();  // Remove oldest value if window exceeds size
    }
    return sum / window.size();                // Return the average
}


void ee_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    if(!eefirstTime){
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
	thetach = yawee; //angle of chaser(ee)
	if(eefirstTime){   //initialize the postiion of chaser and target for the first time ONLY
		xE_in = ee_x;
		yE_in = ee_y;
		thetaE_in = thetach; 
		eefirstTime = false;
        std::cout<<"Initial position of end effector is: xe_in: "<<xE_in<<" yE_in: "<<yE_in<<" thetaE_in: "<<thetaE_in<<std::endl;
		}
}


void target_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    if(!targetfirstTime){
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
	thetat = yawt; //angle of chaser(ee)
	if(targetfirstTime){   //initialize the postiion of chaser and target for the first time ONLY
		xt_in = xt;
		yt_in = yt;
		thetat_in = thetat;
		targetfirstTime = false;
        std::cout<<"Initial position of target is: xt_in: "<<xt_in<<" yt_in: "<<yt_in<<" thetat_in: "<<thetat_in<<std::endl;
		}
}

void base_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    if(!basefirstTime){
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
	theta0 = yawc0; //angle of chaser(ee)
	if(basefirstTime){
		basefirstTime = false;   
		}
}
	



// void lsPosCallback(const std_msgs::Float64::ConstPtr& cmd) { //anti gia ls_position vazo q1
// 	if (abs(cmd->data - q1) > POS_FILTER)
// 		return;
// 	else
// 		q1 = cmd->data;
// }

// void lePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
// 	if (abs(cmd->data - q2) > POS_FILTER)
// 		return;
// 	else
// 		q2 = cmd->data;
// }

// void lsVelCallback(const std_msgs::Float64::ConstPtr& cmd) { //anti gia ls_velocity
// 	if (abs(cmd->data - q1dot) > VEL_FILTER)
// 		return;
// 	else
// 		q1dot = cmd->data;
// }

// void leVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
// 	if (abs(cmd->data - q2dot) > VEL_FILTER)
// 		return;
// 	else
// 		q2dot = cmd->data;
// }

// void rePosCallback(const std_msgs::Float64::ConstPtr& cmd) { //anti gia re_position
// 	if (abs(cmd->data - q3) > POS_FILTER)
// 		return;
// 	else
// 		q3 = cmd->data;
// }

// void reVelCallback(const std_msgs::Float64::ConstPtr& cmd) { //anti gia re_veloctiy
// 	if (abs(cmd->data - q3dot) > VEL_FILTER)
// 		return;
// 	else
// 		q3dot = cmd->data;
// }

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
    if(offsetsdone){
        q1 = moving_average(-(cmd->data), q1_window, window_size,sumq1) + offsetq1;
    }
    else{
        q1 = -(cmd->data) + offsetq1;
    }
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
    if(offsetsdone){
        q2 = moving_average(cmd->data, q2_window, window_size,sumq2) + offsetq2;
    }
    else{
        q2 = cmd->data + offsetq2;
    }
    // q2 = cmd->data + offsetq2;
}

void rePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
    // if (firstTimeq3) {
    //     rawq3 = -(cmd->data);
    //     q3 = rawq3 + offsetq3;
    //     firstTimeq3 = false;
    //     }
    // else{
    //     if (abs(-(cmd->data) - rawq3) > qfilter){
    //         //do nothing
    //         }
    //     else{
    //         rawq3 = -(cmd->data); //o encoder tou mou ta diabazei anapoda
    //         q3 = rawq3 +  offsetq3;  
    //         }
    // }
    if(offsetsdone){
        q3 = moving_average(-(cmd->data), q3_window, window_size,sumq3) + offsetq3;
    }
    else{
        q3 = -(cmd->data) + offsetq3;
    }
    // q3 = -(cmd->data) + offsetq3;
}


void lsVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	// if (abs(cmd->data - q1dot) > VEL_FILTER)
	// 	return;
	// else
    q1dot = moving_average(-(cmd->data), q1dot_window, window_size,sumq1dot);
	// q1dot = -(cmd->data);
}


void leVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	// if (abs(cmd->data - q2dot) > VEL_FILTER)
	// 	return;
	// else
    q2dot = moving_average(cmd->data, q2dot_window, window_size,sumq2dot);
	// q2dot = cmd->data;
}


void reVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	// if (abs(cmd->data - q3dot) > VEL_FILTER)
	// 	return;
	// else
    q3dot = moving_average(-(cmd->data), q3dot_window, window_size,sumq3dot);
	// q3dot = -(cmd->data);
}




void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr&msg){
    force_x = abs(msg->wrench.force.x);  //etsi einai mapped apo to bota  filtered.
    // force_y = msg->wrench.force.y;
	// torque_z = msg->wrench.torque.z;
	// fext(0) = force_x;
	// fext(1) = 0;
	// fext(2) = 0;  //den mas xreiazontai, theloume mono x
	// fext(1) = force_y;
	// fext(2) = torque_z; //na ta bgalo apo sxolio
	// fext(0) =0;
	// fext(1) = 0;
	// fext(2) =0; //for testing
    // if(force_y>10){
    //     ROS_INFO("[foros_simcontroller]: force_y detected: %f N \n",force_y);
    // } 
	if(abs(force_x)<0.25){
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