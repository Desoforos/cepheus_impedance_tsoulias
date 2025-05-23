/*Nikiforos Tsoulias October 2024
Afto to node tha trexei prota (mazi me to interface) oste na arxikopoihsei ta links
O kodikas exei parthei apo to alex gripper node alla einai apomonomenos gia na kanei mono afth th douleia
opou right elbow ousiastika einai to left wrist(q3)*/

#include "includes.h"

bool shutdown_requested = false;

void sigintHandler(int sig) {
    ROS_INFO("Shutdown request received. Performing cleanup tasks...");
    shutdown_requested = true;  // Set flag for graceful shutdown
}

double ls_position = 0.0, le_position = 0.0, re_position = 0.0;
// double ls_position_prev, le_position_prev, re_position_prev;
double ls_velocity = 0.0, le_velocity = 0.0, re_velocity = 0.0;
// double ls_velocity_prev, le_velocity_prev, re_velocity_prev;
bool ls_initialized = false;
bool le_initialized = false;
bool re_initialized = false;
bool ls_first_time = true;
bool le_first_time = true;
bool re_first_time = true;
bool start_moving = false;

double q1_init = -60 * (M_PI / 180);
double q2_init = 105 * (M_PI / 180);
double q3_init = 45 * (M_PI / 180);

double Kp = 0.06;
double Kd = 0.006;

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

void rePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - re_position) > POS_FILTER)
		return;
	else
		re_position = cmd->data;
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


void reVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - re_velocity) > VEL_FILTER)
		return;
	else
		re_velocity = cmd->data;
}


void lsLimitCallback(const std_msgs::UInt8::ConstPtr& cmd) {
	if (cmd->data == 1)
		ls_initialized = true;
}


void leLimitCallback(const std_msgs::UInt8::ConstPtr& cmd) {
	if (cmd->data == 1)
		le_initialized = true;
}


void reLimitCallback(const std_msgs::UInt8::ConstPtr& cmd) {
	if (cmd->data == 1)
		re_initialized = true;
}


void startMovingCallback(const std_msgs::Bool::ConstPtr& msg) {
	start_moving = true;
}

double filter_torque(double torq, double prev) {
	if (torq == 0.0){
		// torq = 0.00001;
		torq = 0.00001;
		if (prev < 0.0)
			torq = torq * -1;
		printf("CHANGED ZERO TORQUE\n");
	}
	return torq;
}


int main(int argc, char **argv) {

    int count = 0;


    /* ros init */
    ros::init(argc, argv, "robot_foros_ignition_node");
    ros::NodeHandle nh;

    ros::Publisher ls_torque_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_effort", 1);
	ros::Publisher le_torque_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_effort", 1);
	ros::Publisher re_torque_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_effort", 1);
	ros::Publisher ls_offset_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_offset", 1);
	ros::Publisher le_offset_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_offset", 1);
	ros::Publisher re_offset_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_offset", 1);

	ros::Subscriber ls_pos_sub = nh.subscribe("read_left_shoulder_position", 1, lsPosCallback);
	ros::Subscriber le_pos_sub = nh.subscribe("read_left_elbow_position", 1, lePosCallback);
	ros::Subscriber re_pos_sub = nh.subscribe("read_right_elbow_position", 1, rePosCallback);
	ros::Subscriber ls_vel_sub = nh.subscribe("read_left_shoulder_velocity", 1, lsVelCallback);
	ros::Subscriber le_vel_sub = nh.subscribe("read_left_elbow_velocity", 1, leVelCallback);
	ros::Subscriber re_vel_sub = nh.subscribe("read_right_elbow_velocity", 1, reVelCallback);
	ros::Subscriber ls_limit_sub = nh.subscribe("read_left_shoulder_limit", 1, lsLimitCallback);
	ros::Subscriber le_limit_sub = nh.subscribe("read_left_elbow_limit", 1, leLimitCallback);
	ros::Subscriber re_limit_sub = nh.subscribe("read_right_elbow_limit", 1, reLimitCallback);

	ros::Subscriber start_moving_sub = nh.subscribe<std_msgs::Bool>("start_moving", 1, startMovingCallback);

    ros::Rate loop_rate(200);  //200hz

	double errorq[3];
	double error_qdot[3];
	double torq[3];
	double prev_torq[3];
	double qd[3];
	double qd_dot[3];

	for (int i = 0; i < 3; i++) {
		errorq[i] = 0.0;
		error_qdot[i] = 0.0;
		torq[i] = 0.0;
		prev_torq[i] = 0.0;
		qd[i] = 0.0;
		qd_dot[i] = 0.0;
	}

	double theta0_des = 90 * (M_PI / 180);
	double theta0_desdot = 0.0;

    double qd1_init = 1.328828;
	double qd2_init = 2.22;
	double qd3_init = 1.777316;

	ros::Time curr_time, t_beg = ros::Time::now();
	ros::Duration all_time;
	double prev_secs = 0.0;
	bool ls_moved = false;
	bool re_moved = false;
	double le_position_after_ls_init, re_position_after_ls_init;
    double s, s_dot;

	std_msgs::Float64 torque;
	std_msgs::Float64 offset;
	std_msgs::Float64 temp_msg;
	std_msgs::Float64 _secs;
	bool initialized = false;
	sleep(4);
	double secs, ls_time, le_time, re_time, move_time, docking_time, penetration_time;
    bool freeFirstTime = false;

    while (ros::ok()) {

		for (int i = 0; i < 3; i++) {
			prev_torq[i] = torq[i];
		}

		curr_time = ros::Time::now();
		all_time = curr_time - t_beg;

		secs = all_time.sec + all_time.nsec * pow(10, -9);

		if (!initialized) {
            std::cout<<"initiating intialisation."<<std::endl;
			if (!ls_initialized) {
                std::cout<<"into left shoulder init"<<std::endl;

				error_qdot[0] = 0.08 + ls_velocity;
				error_qdot[1] = -0.0001 - le_velocity;
				error_qdot[2] = -0.0001 + re_velocity;

				errorq[0] = error_qdot[0] * (secs - prev_secs) + errorq[0];
				errorq[1] = error_qdot[1] * (secs - prev_secs) + errorq[1];
				errorq[2] = error_qdot[2] * (secs - prev_secs) + errorq[2];

				torq[0] = - (Kp/5 * error_qdot[0] + Kd/5 * errorq[0]);
				torq[1] = Kp/6 * error_qdot[1] + Kd/6 * errorq[1];
				torq[2] = - (Kp/5 * error_qdot[2] + Kd/5 * errorq[2]);

			} else if (!re_initialized) {
                std::cout<<"left shoulder done, into left wrist"<<std::endl;
				if (!ls_moved) {
					// move ls to pose position
					if (ls_first_time) {
						//calculate and set offset
						offset.data = qd1_init + ls_position;
						ls_offset_pub.publish(offset);

						ls_first_time = false;
						ls_time = secs;
						ls_position = qd1_init;
						le_position_after_ls_init = le_position;
						re_position_after_ls_init = re_position;
					}

					s = (0.000001875 * pow(secs - ls_time, 5)) + (-0.00009375000000000002 * pow(secs - ls_time, 4)) + (0.00125 * pow(secs - ls_time, 3));
					s_dot = (5 * 0.000001875 * pow(secs - ls_time, 4)) + (4 * -0.00009375000000000002 * pow(secs - ls_time, 3)) + (3 * 0.00125  * pow(secs - ls_time, 2));

					qd[0] = qd1_init + (q1_init - qd1_init) * s;
					qd_dot[0] = (q1_init - qd1_init) * s_dot;

					error_qdot[0] = qd_dot[0] - ls_velocity;
					error_qdot[1] = 0 - le_velocity;
					error_qdot[2] = 0 + re_velocity;
					
					errorq[0] = qd[0] - ls_position;
					errorq[1] = le_position_after_ls_init - le_position;
					errorq[2] = re_position_after_ls_init + re_position;

					torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
					torq[1] = Kp/2 * errorq[1] + Kd/2 * error_qdot[1];
					torq[2] = - (Kp/8 * errorq[2] + Kd/8 * error_qdot[2]);

					// this movement is 20 secs
					if (secs - ls_time >= 20.0)
						ls_moved = true;
                        
						
				} else {
  
					error_qdot[0] = 0 - ls_velocity;
					error_qdot[1] = 0 - le_velocity;
					error_qdot[2] = 0.08 + re_velocity;

					errorq[0] = q1_init - ls_position;
					errorq[1] = le_position_after_ls_init - le_position;
					errorq[2] = error_qdot[2] * (secs - prev_secs) + errorq[2];

					torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
					torq[1] = Kp/2 * errorq[1] + Kd/2 * error_qdot[1];
					torq[2] = - (Kp/5 * error_qdot[2] + Kd/5 * errorq[2]);
				}
			} else if (!le_initialized) {
                std::cout<<"left wrist done, into left elbow"<<std::endl;
				if (!re_moved) {
					// move re to pose position
					if (re_first_time) {
						//calculate and set offset
						offset.data = qd3_init + re_position;
						re_offset_pub.publish(offset);

						re_first_time = false;
						re_time = secs;
						re_position = qd3_init;
					}

					s = (0.00006 * pow(secs - re_time, 5)) + (-0.0015 * pow(secs - re_time, 4)) + (0.01 * pow(secs - re_time, 3));
					s_dot = (5 * 0.00006 * pow(secs - re_time, 4)) + (4 * -0.0015 * pow(secs - re_time, 3)) + (3 * 0.01 * pow(secs - re_time, 2));

					qd[2] = qd3_init + (q3_init - qd3_init) * s;
					qd_dot[2] = (q3_init - qd3_init) * s_dot;

					error_qdot[0] = 0 - ls_velocity;
					error_qdot[1] = 0 - le_velocity;
					error_qdot[2] = qd_dot[2] - re_velocity;

					errorq[0] = q1_init - ls_position;
					errorq[1] = le_position_after_ls_init - le_position;
					errorq[2] = qd[2] - re_position;

					torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
					torq[1] = Kp * errorq[1] + Kd * error_qdot[1];
					torq[2] = - (1.5*Kp * errorq[2] + 1.5*Kd * error_qdot[2]);

					// this movement is 10 secs
					if (secs - re_time >= 10.0)
						re_moved = true;

				} else {
						
					error_qdot[0] = 0 - ls_velocity;
					error_qdot[1] = 0.08 - le_velocity;
					error_qdot[2] = 0 - re_velocity;

					errorq[0] = q1_init - ls_position;
					errorq[1] = error_qdot[1] * (secs - prev_secs) + errorq[1];
					errorq[2] = q3_init - re_position;

					torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
					torq[1] = Kp/6 * error_qdot[1] + Kd/6 * errorq[1];
					torq[2] = - (1.5*Kp * errorq[2] + 1.5*Kd * error_qdot[2]);
				}
			} else {
				// move re to pose position
				if (le_first_time) {
					//calculate and set offset
					offset.data = qd2_init - le_position;
					le_offset_pub.publish(offset);

					le_first_time = false;
					le_time = secs;
					le_position = qd2_init;
				}

				s = (0.00006 * pow(secs - le_time, 5)) + (-0.0015 * pow(secs - le_time, 4)) + (0.01 * pow(secs - le_time, 3));
				s_dot = (5 * 0.00006 * pow(secs - le_time, 4)) + (4 * -0.0015 * pow(secs - le_time, 3)) + (3 * 0.01 * pow(secs - le_time, 2));

				qd[1] = qd2_init + (q2_init - qd2_init) * s;
				qd_dot[1] = (q2_init - qd2_init) * s_dot;

				error_qdot[0] = 0 - ls_velocity;
				error_qdot[1] = qd_dot[1] - le_velocity;
				error_qdot[2] = 0 - re_velocity;

				errorq[0] = q1_init - ls_position;
				errorq[1] = qd[1] - le_position;
				errorq[2] = q3_init - re_position;

				torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
				torq[1] = 2*Kp * errorq[1] + 2*Kd * error_qdot[1];
				torq[2] = - (1.5*Kp * errorq[2] + 1.5*Kd * error_qdot[2]);

				// this movement is 10 secs
				if (secs - le_time >= 10.0)
					initialized = true;
			}
		} else if (!start_moving) {
			// stay there until further notice
			// ROS_INFO("WAITING FOR START MOVING");
			error_qdot[0] = 0 - ls_velocity;
			error_qdot[1] = 0 - le_velocity;
			error_qdot[2] = 0 - re_velocity;

			errorq[0] = q1_init - ls_position;
			errorq[1] = q2_init - le_position;
			errorq[2] = q3_init - re_position;
			// ROS_INFO("ERRORQ:  %f   %f   %f", errorq[0], errorq[1], errorq[2]);
			// ROS_INFO("pos:  %f   %f   %f", ls_position, le_position,re_position);

			torq[0] = - (1.5*Kp * errorq[0] + 1.5*Kd * error_qdot[0]);
			torq[1] = 1.5*Kp * errorq[1] + 1.5*Kd * error_qdot[1];
			torq[2] = - (1.5*Kp * errorq[2] + 1.5*Kd * error_qdot[2]);

		}
        if(start_moving){//kane edo oti thes
            if(!freeFirstTime){
                ROS_INFO("You can move freely the robot now.");
                freeFirstTime = true;
                torque.data = 0.00001;
                ls_torque_pub.publish(torque);
                le_torque_pub.publish(torque);
                re_torque_pub.publish(torque);
            }
            if(count%100 == 0){
                std::cout<< "q1 is: "<<ls_position<<std::endl;
                std::cout<< "q2 is: "<<le_position<<std::endl;
                std::cout<< "q3 is: "<<re_position<<std::endl;
                std::cout<<"////////////"<<std::endl;
            }
            count++;
        }
        //afou ginei opoios elegxos kai ypologistei to torque vector, irthe i ora na to steiloume
        //sto interface node, to opoio tha steilei ta reymata
        if(!start_moving){ //den thelo na kanei interfere me tous publishers apo ton controller mou
            torque.data = filter_torque(torq[0], prev_torq[0]);
            torque.data = torq[0];
            ls_torque_pub.publish(torque);
            torque.data = filter_torque(torq[1], prev_torq[1]);
            torque.data = torq[1];
            le_torque_pub.publish(torque);
            torque.data = filter_torque(torq[2], prev_torq[2]);
            torque.data = torq[2];
            re_torque_pub.publish(torque);
        }

        ros::spinOnce();
		loop_rate.sleep();
    }
    return 0;
}