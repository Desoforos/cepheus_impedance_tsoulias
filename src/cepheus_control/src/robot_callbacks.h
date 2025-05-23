#ifndef ROBOT_CALLBACKS_H
#define ROBOT_CALLBACKS_H
#include "includes.h"
#include "robot_variables.h"

/////////////// CALLBACK FUNCTIONS DEFINITION START////////////////////////




// double moving_average(double new_value, std::deque<double>& window, int size, double& sum) {
//     sum += new_value;
//     window.push_back(new_value);               // Add the new value
//     if (window.size() > size){
//         sum -= window[0];
//         window.pop_front();  // Remove oldest value if window exceeds size
//     }
//     return sum / window.size();                // Return the average
// }

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
            // thetach = moving_average(thetach, thetawindow, 11,sumtheta);
        }
        // ee_x = moving_average(msg->transform.translation.x, xwindow, 11,sumx);
        // ee_y = moving_average(msg->transform.translation.y, ywindow, 11,sumy);
    }
}


void target_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    if(!targetcheck){
        targetcheck = true;
    }
    if(!firstTime){
        // rawxt_prev = rawxt;
        // rawyt_prev = rawyt;
        // rawthetat_prev = rawthetat;
        xt_prev = xt;
        yt_prev = yt;
        thetat_prev = thetat;
    }
	// rawxt = msg->transform.translation.x;
	// rawyt = msg->transform.translation.y;
    xt = msg->transform.translation.x;// - 0.06;  //an to exo orizontia, 6cm, allios ftiakse neo frame gia to target me marker sto interface
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
        // rawthetat = yawt;
        thetat = yawt;
    }
    else{
        if(abs(yawt - thetat) < 8*M_PI/180){
            // rawthetat = yawt; //angle of chaser(ee)
            thetat = yawt; //angle of chaser(ee)

        }
    }
    // thetat = 0; //orizontia
	// thetat = yawt; //angle of chaser(ee)
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
            theta0 = yawc0; //angle of chaser(ee)
            // theta0 = moving_average(theta0, theta0window, 10,sumtheta0);
        }
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
        q1 = moving_average(-(cmd->data)+ offsetq1, q1_window, 10,sumq1);
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
        q2 = moving_average(cmd->data + offsetq2, q2_window, 10,sumq2);
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
        q3 = moving_average(-(cmd->data)+ offsetq3, q3_window, 10,sumq3);
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
    if(offsetsdone){
        q1dot = moving_average(-(cmd->data), q1dot_window, 10,sumq1dot);
    }
    else{
        q1dot = -(cmd->data);
    }
	// q1dot = -(cmd->data);
}


void leVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	// if (abs(cmd->data - q2dot) > VEL_FILTER)
	// 	return;
	// else
    if(offsetsdone){
        q2dot = moving_average(cmd->data, q2dot_window, 10,sumq2dot);
    }
    else{
        q2dot = cmd->data;
    }
	// q2dot = cmd->data;
}


void reVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	// if (abs(cmd->data - q3dot) > VEL_FILTER)
	// 	return;
	// else
    if(offsetsdone){
        q3dot = moving_average(-(cmd->data), q3dot_window, 10,sumq3dot);
    }
    else{
        q3dot = -(cmd->data);
    }
	// q3dot = -(cmd->data);
}




void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr&msg){
    raw_force_x = (msg->wrench.force.z);  //etsi einai mapped apo to bota  filtered.
    // raw_force_x = moving_average(raw_force_x, force_window, 5, forcesum);
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
    // force_x = moving_average(raw_force_x, force_window, force_window_size, forcesum);
    // force_x = raw_force_x; //ase to filtro giati den einai kalo stin arxi
    force_x = 0; //bypass ginetai sto functions
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


void forosCallback(const std_msgs::Bool::ConstPtr&msg){
    // if(msg->data){
    //     beginGrab = true; 
    // }
    stopMotors = true;
}

void acc3_xCallback(const std_msgs::Float32::ConstPtr&msg){
    acc3_x = msg->data;
}

void acc3_yCallback(const std_msgs::Float32::ConstPtr&msg){
    acc3_y = msg->data;
}

void acc3_zCallback(const std_msgs::Float32::ConstPtr&msg){
    acc3_z = msg->data;
}

void acc2_xCallback(const std_msgs::Float32::ConstPtr&msg){
    acc2_x = msg->data;
}

void acc2_yCallback(const std_msgs::Float32::ConstPtr&msg){
    acc2_y = msg->data;
}

void acc2_zCallback(const std_msgs::Float32::ConstPtr&msg){
    acc2_z = msg->data;
}

void acc1_xCallback(const std_msgs::Float32::ConstPtr&msg){
    acc1_x = msg->data;
}

void acc1_yCallback(const std_msgs::Float32::ConstPtr&msg){
    acc1_y = msg->data;
}

void acc1_zCallback(const std_msgs::Float32::ConstPtr&msg){
    acc1_z = msg->data;
}

void imu_linear_accCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    imu_acc_x = msg->vector.x;
    imu_acc_y = msg->vector.y;
}

void imu_angular_velCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    imu_vel_theta = msg->vector.z;
}
/////////////// CALLBACK FUNCTIONS DEFINITION END////////////////////////
#endif