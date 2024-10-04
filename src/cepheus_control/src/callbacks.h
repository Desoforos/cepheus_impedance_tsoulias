#include "includes.h"
#include "variables.h"

/////////////// CALLBACK FUNCTIONS DEFINITION START////////////////////////

void gazeboposCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){ //update the current position of ee and ring
	int i;
	for(i=0; i<msg->name.size(); i++){
		//  ROS_INFO("[Gazebo Callback] Link Name: %s", msg->name[i]);
		if(msg->name[i] == "cepheus::left_grip"){
			tf::Quaternion qee( //for angle of ee
				msg->pose[i].orientation.x,
				msg->pose[i].orientation.y,
				msg->pose[i].orientation.z,
				msg->pose[i].orientation.w);
    		tf::Matrix3x3 m_ee(qee);	
    		double rollee, pitchee, yawee;
			m_ee.getRPY(rollee, pitchee, yawee);
			thetach = yawee; //angle of chaser(ee)
			ee_x = msg->pose[i].position.x; // + (l3+r3)*cos(thetach);// +(l3+r3)*cos(q1+q2+q3);//left grip pose[6]
    		ee_y = msg->pose[i].position.y; // + (l3+r3)*sin(thetach);// + (l3+r3)*sin(q1+q2+q3);
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
				msg->pose[i].orientation.x,
				msg->pose[i].orientation.y,
				msg->pose[i].orientation.z,
				msg->pose[i].orientation.w);
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
				msg->pose[i].orientation.x,
				msg->pose[i].orientation.y,
				msg->pose[i].orientation.z,
				msg->pose[i].orientation.w);
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

/* useless
void ee_target_posCallback(const geometry_msgs::Pose::ConstPtr& msg){
    ROS_INFO("[foros_simcontroller]: pose msg received! ee_target.x: %f ee_target.y: %f ee_target.z: %f \n",msg->position.x, msg->position.y, msg->position.z);
    start_movement=true;
    ee_x_des = msg->position.x;
    ee_y_des=msg->position.y;
    //ee_z=msg->position.z;
}
*/

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){
	int i;
    //ROS_INFO("[foros_simcontroller]: Joint state received! q1: %f q2: %f q1dot: %f q2dot: %f \n",msg->position[1], msg->position[0], msg-> velocity[1], msg-> velocity[0]);
	for(i=0; i<msg->name.size(); i++){
		// ROS_INFO("[Gazebo Callback] Joint Name: %s", msg->name[i]);
		if(msg->name[i] == "left_shoulder_joint"){
			q1 = msg->position[i];
			q1dot = msg->velocity[i];
			// q1 = q1; //evgala to +q01 giati to evala stous ypologismous
		}
		else if(msg->name[i] == "left_elbow_joint"){
			q2 = msg->position[i];
			q2dot = msg->velocity[i];
		}
		else if(msg->name[i] == "left_wrist_joint"){
			q3 = msg->position[i];
			q3dot = msg->velocity[i];
		}
	}
	// std::cout<<"[Joint states callback] q1dot is: "<<q1dot<<std::endl;
	// std::cout<<"[Joint states callback] q2dot is: "<<q2dot<<std::endl;
	// std::cout<<"[Joint states callback] q3dot is: "<<q3dot<<std::endl;

	//theta0 	= msg->position[3]; //reaction wheel MALLON OXI DEN TO THELEI APO RW ALLA APO BASE ORIENTATION
	//theta0dot = msg->velocity[3];
}



/*took from my boy alex*/
/*
void lsPosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - ls_position) > POS_FILTER)
		return;
	else
		ls_position = cmd->data;
}
*/

/*void lePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - le_position) > POS_FILTER)
		return;
	else
		le_position = cmd->data;
}


void lsVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - ls_velocity) > VEL_FILTER)
		return;
	else
		ls_velocity = cmd->data;
}


void leVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - le_velocity) > VEL_FILTER)
		return;
	else
		le_velocity = cmd->data;
}*/
/*thanks alex*/

void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr&msg){
    force_x = msg->wrench.force.x;
    force_y = msg->wrench.force.y;
	torque_z = msg->wrench.torque.z;
	fext(0) = force_x;
	fext(1) = force_y;
	fext(2) = torque_z; //na ta bgalo apo sxolio
	// fext(0) =0;
	// fext(1) = 0;
	// fext(2) =0; //for testing
    // if(force_y>10){
    //     ROS_INFO("[foros_simcontroller]: force_y detected: %f N \n",force_y);
    // } 

}


/////////////// CALLBACK FUNCTIONS DEFINITION END////////////////////////