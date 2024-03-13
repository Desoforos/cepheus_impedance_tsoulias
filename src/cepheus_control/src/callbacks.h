#include "includes.h"
#include "variables.h"

/////////////// CALLBACK FUNCTIONS DEFINITION START////////////////////////

void gazeboposCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){ //update the current position of ee and ring
    ee_x = msg->pose[4].position.x;
    ee_y = msg->pose[4].position.y;
    xt= msg->pose[7].position.x; //ringx
    yt = msg->pose[7].position.y; //ringy
	xc0 = msg->pose[1].position.x; //pose of base
	yc0 = msg->pose[1].position.y;
	xc0dot = msg->twist[1].linear.x; //velocity of base 
	yc0dot = msg->twist[1].linear.y;
	tf::Quaternion q( //for angle of base
        msg->pose[1].orientation.x,
        msg->pose[1].orientation.y,
        msg->pose[1].orientation.z,
        msg->pose[1].orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
	theta0 = yaw; //angle of base
	theta0dot = msg->twist[1].angular.z; //angledot of base
	//////kai gia stoxo
	tf::Quaternion qring(
        msg->pose[7].orientation.x,
        msg->pose[7].orientation.y,
        msg->pose[7].orientation.z,
        msg->pose[7].orientation.w);
    tf::Matrix3x3 mring(qring);
    double rollring, pitchring, yawring;
    mring.getRPY(rollring, pitchring, yawring);
	thetat = yawring; //angle of base
	tf::Quaternion qee( //for angle of base
        msg->pose[4].orientation.x,
        msg->pose[4].orientation.y,
        msg->pose[4].orientation.z,
        msg->pose[4].orientation.w);
    tf::Matrix3x3 m_ee(qee);	
    double rollee, pitchee, yawee;
    m_ee.getRPY(rollee, pitchee, yawee);
	thetach = yawee; //angle of chaser(ee)
	if(firstTime){   //initialize the postiion of chaser and target for the first time ONLY
		xch_in = ee_x;
		ych_in = ee_y;
		xt_in = xt;
		yt_in = yt;
		thetach_in = thetach;
		thetat_in = thetat;
	}
	firstTime = false;
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
    //ROS_INFO("[foros_simcontroller]: Joint state received! q1: %f q2: %f q1dot: %f q2dot: %f \n",msg->position[1], msg->position[0], msg-> velocity[1], msg-> velocity[0]);
    q1      = msg->position[1]; //shoulder
    q2      = msg->position[0]; //elbow
    q1dot   = msg->velocity[1];
    q2dot   = msg->velocity[0];	
	q3 		= msg->position[2]; //wrist
	q3dot 	= msg->velocity[2];
	//theta0 	= msg->position[3]; //reaction wheel MALLON OXI DEN TO THELEI APO RW ALLA APO BASE ORIENTATION
	//theta0dot = msg->velocity[3];
}



/*took from my boy alex*/
void lsPosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - ls_position) > POS_FILTER)
		return;
	else
		ls_position = cmd->data;
}


void lePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
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
}
/*thanks alex*/

void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr&msg){
    force_x = msg->wrench.force.x;
    force_y = msg->wrench.force.y;
	torque_z = msg->wrench.torque.z;
	fext[0] = force_x;
	fext[1] = force_y;
	fext[2] = torque_z;
    // if(force_y>10){
    //     ROS_INFO("[foros_simcontroller]: force_y detected: %f N \n",force_y);
    // } 

}


/////////////// CALLBACK FUNCTIONS DEFINITION END////////////////////////