#ifndef ROBOT_CALLBACKS_H
#define ROBOT_CALLBACKS_H
#include "includes.h"
#include "robot_variables.h"

/////////////// CALLBACK FUNCTIONS DEFINITION START////////////////////////
// geometry_msgs/TransformStamped

void ee_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
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
		xee(0) = ee_x;
		xee(1) = ee_y;
		xee(2) = thetach;
		if(eefirstTime){   //initialize the postiion of chaser and target for the first time ONLY
			xE_in = ee_x;
			yE_in = ee_y;
			thetaE_in = thetach; 
			eefirstTime = false;
		}
}


void target_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
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
		}
}
	

/*
void gazeboposCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){ //update the current position of ee and ring
	int i;
	for(i=0; i<msg->name.size(); i++){
		//  ROS_INFO("[Gazebo Callback] Link Name: %s", msg->name[i]);
		if(msg->name[i] == "cepheus::final_link"){
			tf::Quaternion qee( //for angle of ee
				msg->transform.rotation.x,
				msg->transform.rotation.y,
				msg->transform.rotation.z,
				msg->transform.rotation.w);
    		tf::Matrix3x3 m_ee(qee);	
    		double rollee, pitchee, yawee;
			m_ee.getRPY(rollee, pitchee, yawee);
			thetach = yawee; //angle of chaser(ee)
			ee_x = msg->pose[i].position.x;// + (l3+r3)*cos(q3); den xreiazetai eftiaksa dummy link
    		ee_y = msg->pose[i].position.y;// + (l3+r3)*sin(q3); 
			xee(0) = ee_x;
			xee(1) = ee_y;
			xee(2) = thetach;
			xeedot(0) = msg->twist[i].linear.x;
			xeedot(1) = msg->twist[i].linear.y;
			xeedot(2) = msg->twist[i].angular.z;
			// msg_xee_x.data = xee(0);
			// msg_xee_y.data = xee(1);
			// msg_xee_theta.data = xee(2);
			// std::cout<<"[Gazebo callback] ee x is: "<<ee_x<<std::endl;
			// std::cout<<"[Gazebo callback] ee y is: "<<ee_y<<std::endl;
			// std::cout<<"[Gazebo callback] ee theta is: "<<thetach<<std::endl;
		}
		if(msg->name[i] == "simple_ring::base_link"){
			xt= msg->pose[i].position.x; // - offset - sd; //targetx minus the offset of the cube minus a safety distance
    		yt = msg->pose[i].position.y; //targety
			xtdot = msg->twist[i].linear.x;
			ytdot = msg->twist[i].linear.y;
			thetatdot = msg->twist[i].angular.z;
			tf::Quaternion qring(
				msg->transform.rotation.x,
				msg->transform.rotation.y,
				msg->transform.rotation.z,
				msg->transform.rotation.w);
			tf::Matrix3x3 mring(qring);
			double rollring, pitchring, yawring;
			mring.getRPY(rollring, pitchring, yawring);
			thetat = yawring; //angle of target
			// msg_xt_x.data = xt;
			// msg_xt_y.data = yt;
			// msg_xt_theta.data = thetat;
		}
		if(msg->name[i] == "cepheus::cepheus_base"){
			xc0 = msg->pose[i].position.x; //pose of base
			yc0 = msg->pose[i].position.y;
			xc0dot = msg->twist[i].linear.x; //velocity of base 
			yc0dot = msg->twist[i].linear.y;
			tf::Quaternion q( //for angle of base
				msg->transform.rotation.x,
				msg->transform.rotation.y,
				msg->transform.rotation.z,
				msg->transform.rotation.w);
			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			theta0 = yaw; //angle of base
			theta0dot = msg->twist[i].angular.z; //angledot of base
			// std::cout<<"[Gazebo callback] theta0dot is: "<<theta0dot<<std::endl;
		}
	}
	if(firstTime){   //initialize the postiion of chaser and target for the first time ONLY
		xch_in = ee_x;
		ych_in = ee_y;
		xt_in = xt;
		yt_in = yt;
		thetach_in = thetach;
		thetat_in = thetat;// - M_PI/4; //gia na yparxei mia diafora hehe
		// x_target_in = xt;
		// y_target_in = yt;
		xE_contact = xt;
		yE_contact = yt;
		thetaE_contact = thetat;// - M_PI/2;
		//theta_target_in = thetat;
		xE_in = ee_x;
		yE_in = ee_y;
		thetaE_in = thetach; //ousiastika to egrapsa 2 fores, useless
		theta0in = theta0;
		theta0fin = theta0;
		firstTime = false;
		// msg_xt_x.data = xt_in;
    	// msg_xt_y.data = yt_in;
    	// msg_xt_theta.data = thetat_in;
		ROS_INFO("[callbacks]: First positions have been recorded (xE_in etc). \n");
	}
}
*/


void lsPosCallback(const std_msgs::Float64::ConstPtr& cmd) { //anti gia ls_position vazo q1
	if (abs(cmd->data - q1) > POS_FILTER)
		return;
	else
		q1 = cmd->data;
}

void lePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - q2) > POS_FILTER)
		return;
	else
		q2 = cmd->data;
}

void lsVelCallback(const std_msgs::Float64::ConstPtr& cmd) { //anti gia ls_velocity
	if (abs(cmd->data - q1dot) > VEL_FILTER)
		return;
	else
		q1dot = cmd->data;
}

void leVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - q2dot) > VEL_FILTER)
		return;
	else
		q2dot = cmd->data;
}

void rePosCallback(const std_msgs::Float64::ConstPtr& cmd) { //anti gia re_position
	if (abs(cmd->data - q3) > POS_FILTER)
		return;
	else
		q3 = cmd->data;
}

void reVelCallback(const std_msgs::Float64::ConstPtr& cmd) { //anti gia re_veloctiy
	if (abs(cmd->data - q3dot) > VEL_FILTER)
		return;
	else
		q3dot = cmd->data;
}



void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr&msg){
    force_x = msg->wrench.force.x;  //etsi einai mapped apo to bota  filtered.
    // force_y = msg->wrench.force.y;
	// torque_z = msg->wrench.torque.z;
	fext(0) = force_x;
	fext(1) = 0;
	fext(2) = 0;  //den mas xreiazontai, theloume mono x
	// fext(1) = force_y;
	// fext(2) = torque_z; //na ta bgalo apo sxolio
	// fext(0) =0;
	// fext(1) = 0;
	// fext(2) =0; //for testing
    // if(force_y>10){
    //     ROS_INFO("[foros_simcontroller]: force_y detected: %f N \n",force_y);
    // } 
	if(abs(fext(0))<0.5){
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