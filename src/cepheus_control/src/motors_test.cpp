#include "includes.h"

bool shutdown_requested = false;
double th0,th1,th2,th3;  //oi gonies ton frames se sxesh me adraneiako ss, tha metatrapoun se q1,q2,q3
double q1, q2, q3;
double q1dot, q2dot, q3dot;
double q1prev,q2prev,q3prev;
bool firstTime = true;

void sigintHandler(int sig) {
    ROS_INFO("Shutdown request received. Performing cleanup tasks...");
    shutdown_requested = true;  // Set flag for graceful shutdown
}

void calcAngles(){
    /*na ftiakso katallhla frames sth vicon gia na ypologizo q1,q2,q3*/
    /*fadazomai: q1 = gonia frame 1-gonia bashs(theta0)
    q2 = gonia frame2 - gonia frame1
    q3 = gonia frame3(end effector) - gonia frame2*
    opou: frame1: sto arm, frame2: sto forearm, frame3: sto gripper */
    if(firstTime){
        q1 = th1-th0;
        q2 = th2-th1;
        q3 = th3-th2;
        q1prev = q1;
        q2prev = q2;
        q3prev = q3;
        firstTime = false;
    }
    else{
    q1prev = q1;
    q2prev = q2;
    q3prev = q3;
    q1 = th1-th0;
    q2 = th2-th1;
    q3 = th3-th2;
    }
}

void updateVel(double dt){
    q1dot = (q1-q1prev)/dt;
    q2dot = (q2-q2prev)/dt;
    q3dot = (q3-q3prev)/dt;
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

void base_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
	tf::Quaternion qee( //for angle of ee
		msg->transform.rotation.x,
		msg->transform.rotation.y,
		msg->transform.rotation.z,
		msg->transform.rotation.w);
    tf::Matrix3x3 m_ee(qee);	
    double rollee, pitchee, yawee;
	m_ee.getRPY(rollee, pitchee, yawee);
	th0 = yawee; 
}

void ls_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
	tf::Quaternion qee( 
		msg->transform.rotation.x,
		msg->transform.rotation.y,
		msg->transform.rotation.z,
		msg->transform.rotation.w);
    tf::Matrix3x3 m_ee(qee);	
    double rollee, pitchee, yawee;
	m_ee.getRPY(rollee, pitchee, yawee);
	th1 = yawee; 
}

void le_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
	tf::Quaternion qee( 
		msg->transform.rotation.x,
		msg->transform.rotation.y,
		msg->transform.rotation.z,
		msg->transform.rotation.w);
    tf::Matrix3x3 m_ee(qee);	
    double rollee, pitchee, yawee;
	m_ee.getRPY(rollee, pitchee, yawee);
	th2 = yawee; 
}

void ee_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
	tf::Quaternion qee( //for angle of ee
		msg->transform.rotation.x,
		msg->transform.rotation.y,
		msg->transform.rotation.z,
		msg->transform.rotation.w);
    tf::Matrix3x3 m_ee(qee);	
    double rollee, pitchee, yawee;
	m_ee.getRPY(rollee, pitchee, yawee);
	th3 = yawee; 
}

void lsPosCallback(const std_msgs::Float64::ConstPtr& cmd) {
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

void rePosCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - q3) > POS_FILTER)
		return;
	else
		q3 = cmd->data;
}


void lsVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
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


void reVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	if (abs(cmd->data - q3dot) > VEL_FILTER)
		return;
	else
		q3dot = cmd->data;
}






int main(int argc, char **argv) {



    /* ros init */
    ros::init(argc, argv, "motors_test_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);

    ros::Publisher ls_torque_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_effort", 1);   //ropi se q1
	ros::Publisher le_torque_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_effort", 1);      //ropi se q2
  	ros::Publisher re_torque_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_effort", 1);     //ropi se q3 (ki omos to right elbow ousiastika einai to wrist)
    ros::Publisher rw_torque_pub = nh.advertise<std_msgs::Float64>("set_reaction_wheel_effort", 1);  //ropi se reaction wheel    (ola afta ta akouei to interface)
    int count = 0;
    ros::Rate loop_rate(200);  //200hz
    std_msgs::Float64 torque;
  


	// ros::Publisher ls_offset_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_offset", 1);
	// ros::Publisher le_offset_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_offset", 1);
    // ros::Publisher re_offset_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_offset", 1);

    
    // ros::Subscriber base_pos_sub = nh.subscribe("/vicon/base/base", 1, base_posCallback); 
    // ros::Subscriber ls_pos_sub = nh.subscribe("/vicon/frame1/frame1", 1, ls_posCallback);
    // ros::Subscriber le_pos_sub = nh.subscribe("/vicon/frame2/frame2", 1, le_posCallback);
    // ros::Subscriber ee_pos_sub = nh.subscribe("/vicon/end_effector_new/end_effector_new", 1, ee_posCallback);

    ros::Subscriber ls_pos_sub = nh.subscribe("read_left_shoulder_position", 1, lsPosCallback);
	ros::Subscriber le_pos_sub = nh.subscribe("read_left_elbow_position", 1, lePosCallback);
	ros::Subscriber re_pos_sub = nh.subscribe("read_right_elbow_position", 1, rePosCallback);
	ros::Subscriber ls_vel_sub = nh.subscribe("read_left_shoulder_velocity", 1, lsVelCallback);
	ros::Subscriber le_vel_sub = nh.subscribe("read_left_elbow_velocity", 1, leVelCallback);
	ros::Subscriber re_vel_sub = nh.subscribe("read_right_elbow_velocity", 1, reVelCallback);
    

    double q1des, q2des, q3des;
    char cmd;
    bool initialiseArm = false;
    double Kp = 0.06;
    double Kd = 0.006;

	double errorq[3];
	double errorqdot[3];
	double torq[3];
	double prev_torq[3];
	double qd[3];
	double qd_dot[3];

	for (int i = 0; i < 3; i++) {
		errorq[i] = 0.0;
		errorqdot[i] = 0.0;
		torq[i] = 0.0;
		prev_torq[i] = 0.0;
		qd[i] = 0.0;
		qd_dot[i] = 0.0;
	}

    // th0 = th1 = th2 = th3 = 0;  //arxikopoihsh kalou kakou
    ros::spinOnce();
    // calAngles();
    // updateVel(0.005); //200hz
    std::cout<<"q1 is: "<<(q1*180/M_PI)<<" degrees."<<std::endl;
    std::cout<<"q2 is: "<<(q2*180/M_PI)<<" degrees."<<std::endl;
    std::cout<<"q3 is: "<<(q3*180/M_PI)<<" degrees."<<std::endl;
    std::cout <<"Give me q1des (degrees): "<<std::endl;
    std::cin >>q1des;
    std::cout <<"Give me q2des (degrees): "<<std::endl;
    std::cin >>q2des;    
    std::cout <<"Give me q3des (degrees): "<<std::endl;
    std::cin >>q3des;

    q1des = q1des*M_PI/180; //from deg to rad
    q2des = q2des*M_PI/180; //from deg to rad
    q3des = q3des*M_PI/180; //from deg to rad

    std::cout<<"Press Y if you want to initialize the arm, or any other key not to. "<<std::endl;
    std::cin>>cmd;

    if(cmd == 'Y'){
        initialiseArm = true;
    }


    while(ros::ok()){
        ros::spinOnce();
        calcAngles();
        if(count%100 == 0){
            std::cout<<"q1 is: "<<(q1*180/M_PI)<<" degrees."<<std::endl;
            std::cout<<"q2 is: "<<(q2*180/M_PI)<<" degrees."<<std::endl;
            std::cout<<"q3 is: "<<(q3*180/M_PI)<<" degrees."<<std::endl;
        }
        if(initialiseArm){
            errorq[0] = q1des - q1;
            errorq[1] = q2des - q2;
            errorq[2] = q3des - q3;

            errorqdot[0] = 0.0 - q1dot;
            errorqdot[1] = 0.0 - q2dot;
            errorqdot[2] = 0.0 - q3dot;

            if(abs(errorq[0])<0.001 && abs(errorq[1])<0.001 && abs(errorq[2])<0.001
                && abs(errorqdot[0])<0.001 && abs(errorqdot[1])<0.001 && abs(errorqdot[2])<0.001 ){
                    ROS_INFO("Initial arm pose established.");
                    initialiseArm = false;
                    torque.data = 0.00001;
                    ls_torque_pub.publish(torque);
                    le_torque_pub.publish(torque);
                    re_torque_pub.publish(torque);
                    break;
                }

            prev_torq[0] = torq[0];
            prev_torq[1] = torq[1];
            prev_torq[2] = torq[2];
            torq[0] = Kp*errorq[0] + Kd*errorqdot[0];
            torq[1] = Kp*errorq[1] + Kd*errorqdot[1];
            torq[2] = Kp*errorq[2] + Kd*errorqdot[2];

            torque.data = filter_torque(torq[0], prev_torq[0]);
            // torque.data = 0.001; //torq[0];
            ls_torque_pub.publish(torque);
            torque.data = filter_torque(torq[1], prev_torq[1]);
            // torque.data = 0.0001; //torq[1];
            le_torque_pub.publish(torque); //sxolio gia debugging
            torque.data = filter_torque(torq[2], prev_torq[2]);
            // torque.data = torq[2];
            re_torque_pub.publish(torque);
        }

        loop_rate.sleep();

    }

    return 0;
}