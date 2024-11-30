#include "includes.h"

bool shutdown_requested = false;
double theta0,th1,th2,th3;  //oi gonies ton frames se sxesh me adraneiako ss, tha metatrapoun se q1,q2,q3
double q1, q2, q3;
double theta0dot,q1dot, q2dot, q3dot;
double theta0prev,q1prev,q2prev,q3prev;
bool firstTime = true;
double offsetq1 = 0.0;
double offsetq2 = 0.0;
double offsetq3 = 0.0;
double rw_max_torque = 0.2;
double a0,a1,a2,a3,a4,a5;
double q1step,q2step,q3step,theta0step;
double q1stepdot,q2stepdot,q3stepdot,theta0stepdot;
double q1in,q2in,q3in,theta0in;
double tfree;
double theta0des,q1des, q2des, q3des;
bool firstTimeq1 = true;
bool firstTimeq2 = true;
bool firstTimeq3 = true;
bool offsetsdone = false;
double rawq1, rawq2, rawq3; //angles before the offset addition
const double qfilter = 5*M_PI/180; //5 moires




void sigintHandler(int sig) {
    ROS_INFO("Shutdown request received. Performing cleanup tasks...");
    shutdown_requested = true;  // Set flag for graceful shutdown
}



void updateVel(double dt){
    theta0dot = (theta0-theta0prev)/dt;
    // q1dot = (q1-q1prev)/dt;
    // q2dot = (q2-q2prev)/dt;
    // q3dot = (q3-q3prev)/dt;
}

double filter_torque(double torq, double prev) { //an einai mhden dinei 0.00001 kai an erxetai apo ta arnhtika dinei -0.00001
	if (torq == 0.0){
		// torq = 0.00001;
		torq = 0.00001;
		if (prev < 0.0)
			torq = torq * -1;
		// printf("CHANGED ZERO TORQUE\n");
	}
	return torq;
}

void gazeboposCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){ //update the current position of ee and ring
	int i;
	for(i=0; i<msg->name.size(); i++){
		//  ROS_INFO("[Gazebo Callback] Link Name: %s", msg->name[i]);
		if(msg->name[i] == "cepheus::cepheus_base"){
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
}
	
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
}



void calculateTrajecotryPolynomials(double tfree){
    Eigen::MatrixXd eq_matrix(3,3);
    Eigen::VectorXd eq_bscale(3);
    // double ts = 0.1*tfree;
    // double wn = 6/ts;
    // kprop_mb = wn*wn;
    // kder_mb = 2*wn; //ta vgala tha ta vro monos mou
    
    
    eq_matrix << pow(tfree,3), pow(tfree,4), pow(tfree,5),
                3*pow(tfree,2), 4*pow(tfree,3), 5*pow(tfree,4),
                6*tfree, 12*pow(tfree,2), 20*pow(tfree,3); 
    
    eq_bscale << 1 , 0, 0;

    Eigen::VectorXd res = eq_matrix.colPivHouseholderQr().solve(eq_bscale);

    a0 = a1 = a2 = 0; //from paper calculations, for t0 = 0
    a3 = res(0);
    a4 = res(1);
    a5 = res(2);
}

void finaltrajectories(double t,double tfree){
  	// if(firstTime){   //initialize the postiion of chaser and target for the first time ONLY
    //     theta0in = theta0;
    //     q1in = q1;
    //     q2in = q2;
    //     q3in = q3;
	// 	ROS_INFO("[Motors test]: First angles have been recorded . \n");
    //     firstTime = false;
	//   }
    double s,sdot, sdotdot;

    s = a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
    sdot = a1 + 2*a2*t + 3*a3*pow(t,2) + 4*a4*pow(t,3) + 5*a5*pow(t,4);
    sdotdot = 2*a2 + 6*a3*t + 12*a4*pow(t,2) + 20*a5*pow(t,3);


    if(t<=tfree){
      q1step = q1in + s*(q1des - q1in);
      q2step = q2in + s*(q2des - q2in);
      q3step = q3in + s*(q3des - q3in);
      theta0step = theta0in + s*(theta0des - theta0in);
      
      q1stepdot =  sdot*(q1des - q1in);
      q2stepdot =  sdot*(q2des - q2in);
      q3stepdot =  sdot*(q3des - q3in);
      theta0stepdot =  sdot*(theta0des - theta0in);

      
    }
    else{
      q1step = q1des;
      q2step = q2des;
      q3step = q3des;
      theta0step = theta0des;
      
      q1stepdot =  0;
      q2stepdot =  0;
      q3stepdot =  0;
      theta0stepdot =  0;
    }
}


int main(int argc, char **argv) {

    /* ros init */
    ros::init(argc, argv, "gazebo_motors_test_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);

    ros::Publisher base_force_pub = nh.advertise<geometry_msgs::Wrench>("/cepheus/force_base_topic", 100); //anti gia 10 gia na doume

    ros::Publisher ls_torque_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_shoulder_effort_controller/command", 100);   //ropi se q1
	ros::Publisher le_torque_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_elbow_effort_controller/command", 100);      //ropi se q2
  	ros::Publisher re_torque_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_wrist_effort_controller/command", 100);     //ropi se q3 (ki omos to right elbow ousiastika einai to wrist)
    ros::Publisher rw_torque_pub = nh.advertise<std_msgs::Float64>("/cepheus/reaction_wheel_effort_controller/command", 100);  //ropi se reaction wheel    (ola afta ta akouei to interface)
    int count = 0;
    ros::Rate loop_rate(200);  //200hz
    std_msgs::Float64 torque;

    
   
    ros::Subscriber joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/cepheus/joint_states",100,jointStatesCallback); //tha to anoikso
    ros::Subscriber gazebo_pos_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",100,gazeboposCallback);     //tha to anoikso


    std_msgs::Float64 msg_torqueq1;
    std_msgs::Float64 msg_torqueq2;
    std_msgs::Float64 msg_torqueq3;
    std_msgs::Float64 msg_torquerw;

    std_msgs::Float64 msg_q1;
    std_msgs::Float64 msg_q2;
    std_msgs::Float64 msg_q3;
    std_msgs::Float64 msg_theta0;

    std_msgs::Float64 msg_q1dot;
    std_msgs::Float64 msg_q2dot;
    std_msgs::Float64 msg_q3dot;
    std_msgs::Float64 msg_theta0dot;

    std_msgs::Float64 msg_q1d;
    std_msgs::Float64 msg_q2d;
    std_msgs::Float64 msg_q3d;
    std_msgs::Float64 msg_theta0d;

    std_msgs::Float64 msg_q1ddot;
    std_msgs::Float64 msg_q2ddot;
    std_msgs::Float64 msg_q3ddot;
    std_msgs::Float64 msg_theta0ddot;
    geometry_msgs::Wrench base_wrench;

    base_wrench.force.x = 0.0;
    base_wrench.force.y = 0.0;
    base_wrench.force.z = 0.0;
    base_wrench.torque.x = 0.0;
    base_wrench.torque.y = 0.0;
    base_wrench.torque.z = 0.0;

    

    char cmd;
    bool initialiseArm = false;
    double Kp = 0.06;
    double Kd = 0.006;

	double errorq[4];
	double errorqdot[4];
	double torq[4];
	double prev_torq[4];
	double qd[4];
	double qd_dot[4];

    double q1known = 1.84447;  //gonies analoga me tin gnosti diataksi, ego ekana omos thetiko akro, agonas karpos arnitika akra
    double q2known = -0.9542;
    double q3known = -0.2856; 

    
    bool legitChar = false;
    ros::Time curr_time, t_beg;
    ros::Duration dur_time;
    double secs;
    bool record = false;


    rosbag::Bag bag;
    std::string path = "/home/desoforos/cepheus_impedance_tsoulias/rosbags/" ;
    std::string bag_file_name;

    

	for (int i = 0; i < 4; i++) {
		errorq[i] = 0.0;
		errorqdot[i] = 0.0;
		torq[i] = 0.0001;
		prev_torq[i] = 0.0001;
		qd[i] = 0.0;
		qd_dot[i] = 0.0;
	}


    ROS_INFO("[Motors test]: You want to record to a bag? Press Y for yes, anything else for no. \n");

    std::cin>>cmd;
    if(cmd == 'Y'){
        record = true;
        ROS_INFO("[Motors test]: Please provide the name of the bag (dont put .bag). \n");
        std::cin >>  bag_file_name;
        bag.open(path + bag_file_name + ".bag", rosbag::bagmode::Write);
    }
    




    ros::spinOnce();
    loop_rate.sleep();

    std::cout<<"theta0 is: "<<(theta0*180/M_PI)<<" degrees."<<std::endl; 
    std::cout<<"q1 is: "<<(q1*180/M_PI)<<" degrees."<<std::endl;
    std::cout<<"q2 is: "<<(q2*180/M_PI)<<" degrees."<<std::endl;
    std::cout<<"q3 is: "<<(q3*180/M_PI)<<" degrees."<<std::endl;

    std::cout <<"Give me q1des (degrees): "<<std::endl;
    std::cin >>q1des;
    std::cout <<"Give me q2des (degrees): "<<std::endl;
    std::cin >>q2des;    
    std::cout <<"Give me q3des (degrees): "<<std::endl;
    std::cin >>q3des;




    while(!legitChar){
        ROS_INFO("[Motors test]:Press Y to initialize arm or N not to. ");
        std::cin >>cmd;
        if(cmd == 'Y'){
            ROS_INFO("[Motors test]: Please provide tfree. ");
            std::cin>>tfree;
            calculateTrajecotryPolynomials(tfree);
            legitChar = true;
            initialiseArm = true;
            ROS_WARN("Initializing arm...");
            ros::spinOnce();
            loop_rate.sleep();
            theta0in = theta0;  //to arxiko theta0, thelo na meinei statheri gonia
            q1des = q1des*M_PI/180; //from deg to rad
            q2des = q2des*M_PI/180; //from deg to rad
            q3des = q3des*M_PI/180; //from deg to rad
            std::cout<<"[In initialization] theta0is: "<<theta0<<std::endl;
            q1in = q1;
            q2in = q2;
            q3in = q3;
            t_beg  = ros::Time::now();
        }
        if(cmd == 'N'){
            legitChar = true;
            initialiseArm = false;
            ROS_WARN("Not initializing arm.");
            }
    }


    // initialiseArm = false;  //gia dokimi
    


    while(ros::ok() && !shutdown_requested){
        ros::spinOnce();
        loop_rate.sleep();
        if(initialiseArm){

            curr_time = ros::Time::now();
		    dur_time = curr_time - t_beg;
            secs = dur_time.sec + dur_time.nsec * pow(10, -9);
            finaltrajectories(secs,tfree);

            // errorq[0] = theta0step - theta0;
            errorq[0] = theta0in - theta0;
            errorq[1] = q1step - q1;
            errorq[2] = q2step - q2;
            errorq[3] = q3step - q3;
            
            // errorqdot[0] = theta0stepdot - theta0dot;
            errorqdot[0] = 0.0 - theta0dot;
            errorqdot[1] = q1stepdot - q1dot;
            errorqdot[2] = q2stepdot - q2dot;
            errorqdot[3] = q3stepdot - q3dot;

            if((secs > tfree+10) && abs(errorq[0])<0.001 && abs(errorq[1])<0.001 && abs(errorq[2])<0.001
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
            prev_torq[3] = torq[3];
            torq[0] = 0.5*errorq[0] + 2*errorqdot[0]; 
            torq[1] = (0.6*errorq[1] + 0.3*errorqdot[1]); 
            torq[2] = (0.6*errorq[2] + 0.3*errorqdot[2]);
            torq[3] = (0.6*errorq[3] + 0.3*errorqdot[3]);


            base_wrench.torque.z = filter_torque(torq[0], prev_torq[0]);
            base_force_pub.publish(base_wrench);
            torque.data = filter_torque(torq[1], prev_torq[1]);
            ls_torque_pub.publish(torque);  //dokimi mono reaction wheel
            torque.data = filter_torque(torq[2], prev_torq[2]);
            le_torque_pub.publish(torque); //dokimi mono rw
            torque.data = filter_torque(torq[3], prev_torq[3]);
            re_torque_pub.publish(torque); //dokimi mono rw

            if(count%100 == 0){
            std::cout<<"///////////////////////////////////"<<std::endl;
            std::cout<<"time is: "<<secs<<" seconds."<<std::endl;
            // std::cout<<"theta0des is: "<<(theta0des*180/M_PI)<<"degrees. "<<std::endl;
            // std::cout<<"theta0 is: "<<(theta0*180/M_PI)<<" degrees. "<<std::endl;
            // std::cout<<"theta0dot is: "<<theta0dot<<std::endl;
            // std::cout<<"theta0prev is: "<<theta0prev<<std::endl;
            // std::cout<<"error is: "<<errorq[0]*180/M_PI<<" degrees. "<<std::endl;
            std::cout<<"theta INITIAL is: "<<theta0in*180/M_PI<<" deg."<<std::endl;
            std::cout<<"theta0  is: "<<theta0*180/M_PI<<" deg."<<std::endl;
            std::cout<<"q1 is: "<<180*q1/M_PI<< " deg"<<std::endl;
            std::cout<<"q2  is: "<<180*q2/M_PI<< " deg"<<std::endl;
            std::cout<<"q3  is: "<<180*q3/M_PI<< " deg"<<std::endl;
            std::cout<<"error theta0  is: "<<errorq[0]<<" deg."<<std::endl;
            std::cout<<"error q1 is: "<<errorq[1]<< " deg"<<std::endl;
            std::cout<<"error q2  is: "<<errorq[2]<< " deg"<<std::endl;
            std::cout<<"error q3  is: "<<errorq[3]<< " deg"<<std::endl;

            std::cout<<"RW torq  is: "<<torq[0]<< " Nm"<<std::endl;
            std::cout<<"q1 torq is: "<<torq[1]<< " Nm"<<std::endl;
            std::cout<<"q2 torq is: "<<torq[2]<< " Nm"<<std::endl;
            std::cout<<"q3 torq is: "<<torq[3]<< " Nm"<<std::endl;



            }
            count++;

            if(record){
                    msg_q1d.data = q1step;
                    msg_q2d.data = q2step;
                    msg_q3d.data = q3step;
                    // msg_theta0d.data = theta0step;
                    msg_theta0d.data = theta0in;

                    msg_q1ddot.data = q1stepdot;
                    msg_q2ddot.data = q2stepdot;
                    msg_q3ddot.data = q3stepdot;
                    // msg_theta0ddot.data = theta0stepdot;  
                    msg_theta0ddot.data = 0.0; 

                    msg_q1.data = q1;
                    msg_q2.data = q2;
                    msg_q3.data = q3;
                    msg_theta0.data = theta0;

                    msg_q1dot.data = q1dot;
                    msg_q2dot.data = q2dot;
                    msg_q3dot.data = q3dot;
                    msg_theta0dot.data = theta0dot;   

                    msg_torquerw.data = torq[0];
                    msg_torqueq1.data = torq[1];
                    msg_torqueq2.data = torq[2];
                    msg_torqueq3.data = torq[3];

                    bag.write("/cepheus/q1d", ros::Time::now(), msg_q1d);
                    bag.write("/cepheus/q2d", ros::Time::now(), msg_q2d);
                    bag.write("/cepheus/q3d", ros::Time::now(), msg_q3d);
                    bag.write("/cepheus/theta0d", ros::Time::now(), msg_theta0d);

                    bag.write("/cepheus/q1ddot", ros::Time::now(), msg_q1ddot);
                    bag.write("/cepheus/q2ddot", ros::Time::now(), msg_q2ddot);
                    bag.write("/cepheus/q3ddot", ros::Time::now(), msg_q3ddot);
                    bag.write("/cepheus/theta0ddot", ros::Time::now(), msg_theta0ddot);  


                    bag.write("/cepheus/q1", ros::Time::now(), msg_q1);
                    bag.write("/cepheus/q2", ros::Time::now(), msg_q2);
                    bag.write("/cepheus/q3", ros::Time::now(), msg_q3);
                    bag.write("/cepheus/theta0", ros::Time::now(), msg_theta0);

                    bag.write("/cepheus/q1dot", ros::Time::now(), msg_q1dot);
                    bag.write("/cepheus/q2dot", ros::Time::now(), msg_q2dot);
                    bag.write("/cepheus/q3dot", ros::Time::now(), msg_q3dot);
                    bag.write("/cepheus/theta0dot", ros::Time::now(), msg_theta0dot);  

                    bag.write("/cepheus/torquerw", ros::Time::now(), msg_torquerw);
                    bag.write("/cepheus/torqueq1", ros::Time::now(), msg_torqueq1);
                    bag.write("/cepheus/torqueq2", ros::Time::now(), msg_torqueq2);
                    bag.write("/cepheus/torqueq3", ros::Time::now(), msg_torqueq3);               
            }
        }
    }

    if(record){
        bag.close();
    }

    return 0;
}