#include "includes.h"
#include <deque>  // For the sliding window

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



std::deque<double> q1_window;  // Stores the last N values
std::deque<double> q2_window;  // Stores the last N values
std::deque<double> q3_window;  // Stores the last N values

std::deque<double> q1dot_window;  // Stores the last N values
std::deque<double> q2dot_window;  // Stores the last N values
std::deque<double> q3dot_window;  // Stores the last N values


const int window_size = 10;     // Size of the sliding window

double sumq1 = 0, sumq2 = 0, sumq3 =0;
double sumq1dot = 0, sumq2dot = 0, sumq3dot =  0;



// double moving_average(double new_value, std::deque<double>& window, int size, double &sum) {
//     sum += new_value;
//     window.push_back(new_value);               // Add the new value
//     if (window.size() > size){
//         sum -= window[0];
//         window.pop_front();  // Remove oldest value if window exceeds size
//     }
//     return sum / window.size();                // Return the average
// }

double moving_average(double new_value, std::deque<double>& window, int size) {
    window.push_back(new_value);               // Add the new value
    if (window.size() > size) window.pop_front();  // Remove oldest value if window exceeds size
    double sum = 0;
    for (double val : window) sum += val;      // Sum all values in the window
    return sum / window.size();                // Return the average
}







void sigintHandler(int sig) {
    ROS_INFO("Shutdown request received. Performing cleanup tasks...");
    shutdown_requested = true;  // Set flag for graceful shutdown
}

// void calcAngles(){
//     /*na ftiakso katallhla frames sth vicon gia na ypologizo q1,q2,q3*/
//     /*fadazomai: q1 = gonia frame 1-gonia bashs(theta0)
//     q2 = gonia frame2 - gonia frame1
//     q3 = gonia frame3(end effector) - gonia frame2*
//     opou: frame1: sto arm, frame2: sto forearm, frame3: sto gripper */
//     if(firstTime){
//         q1 = th1-th0;
//         q2 = th2-th1;
//         q3 = th3-th2;
//         q1prev = q1;
//         q2prev = q2;
//         q3prev = q3;
//         firstTime = false;
//     }
//     else{
//     q1prev = q1;
//     q2prev = q2;
//     q3prev = q3;
//     q1 = th1-th0;
//     q2 = th2-th1;
//     q3 = th3-th2;
//     }
// }

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

void base_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    if(msg == nullptr){
        ROS_WARN("Vicon null pointer, exiting...");
    }
    else{
        tf::Quaternion qee( //for angle of ee
            msg->transform.rotation.x,
            msg->transform.rotation.y,
            msg->transform.rotation.z,
            msg->transform.rotation.w);
        tf::Matrix3x3 m_ee(qee);	
        double rollee, pitchee, yawee;
        m_ee.getRPY(rollee, pitchee, yawee);
        if(firstTime){
            theta0 = yawee;
            theta0prev = theta0;
            firstTime = false;
        }
        else{
            if((abs(theta0 - yawee)>10*M_PI/180)){
                //do nothing
            }
            else{
                theta0prev = theta0;
                theta0 = yawee;
            }
        }
    }
	// theta0 = yawee; 
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
        q1 = moving_average(-(cmd->data), q1_window, window_size) + offsetq1;
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
        q2 = moving_average(cmd->data, q2_window, window_size) + offsetq2;
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
        q3 = moving_average(-(cmd->data), q3_window, window_size) + offsetq3;
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
    q1dot = moving_average(-(cmd->data), q1dot_window, window_size);
	// q1dot = -(cmd->data);
}


void leVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	// if (abs(cmd->data - q2dot) > VEL_FILTER)
	// 	return;
	// else
    q2dot = moving_average(cmd->data, q2dot_window, window_size);
	// q2dot = cmd->data;
}


void reVelCallback(const std_msgs::Float64::ConstPtr& cmd) {
	// if (abs(cmd->data - q3dot) > VEL_FILTER)
	// 	return;
	// else
    q3dot = moving_average(-(cmd->data), q3dot_window, window_size);
	// q3dot = -(cmd->data);
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

    theta0 = theta0dot = theta0prev = 0.0001;
    // theta0 = q1 = q2 = q3 = q1dot = q2dot = q3dot = theta0dot = 0; //apla gia dokimi




    /* ros init */
    ros::init(argc, argv, "motors_test_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);

    ros::Publisher ls_torque_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_effort", 1);   //ropi se q1
	ros::Publisher le_torque_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_effort", 1);      //ropi se q2
  	ros::Publisher re_torque_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_effort", 1);     //ropi se q3 (ki omos to right elbow ousiastika einai to wrist)
    ros::Publisher rw_torque_pub = nh.advertise<std_msgs::Float64>("cmd_torque", 1);  //ropi se reaction wheel    (ola afta ta akouei to interface)
    int count = 0;
    ros::Rate loop_rate(100);  //100hz
    std_msgs::Float64 torque;
  


	// ros::Publisher ls_offset_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_offset", 1);
	// ros::Publisher le_offset_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_offset", 1);
    // ros::Publisher re_offset_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_offset", 1);

    
    ros::Subscriber base_pos_sub = nh.subscribe("/vicon/cepheusbase/cepheusbase", 10, base_posCallback); //TO ANOIGOKLEINO
    

    // ros::Subscriber ls_pos_sub = nh.subscribe("/vicon/frame1/frame1", 1, ls_posCallback);
    // ros::Subscriber le_pos_sub = nh.subscribe("/vicon/frame2/frame2", 1, le_posCallback);
    // ros::Subscriber ee_pos_sub = nh.subscribe("/vicon/end_effector_new/end_effector_new", 1, ee_posCallback);

    ros::Subscriber ls_pos_sub = nh.subscribe("read_left_shoulder_position", 10, lsPosCallback);
	ros::Subscriber le_pos_sub = nh.subscribe("read_left_elbow_position", 10, lePosCallback);
	ros::Subscriber re_pos_sub = nh.subscribe("read_right_elbow_position", 10, rePosCallback);
	ros::Subscriber ls_vel_sub = nh.subscribe("read_left_shoulder_velocity", 10, lsVelCallback);
	ros::Subscriber le_vel_sub = nh.subscribe("read_left_elbow_velocity", 10, leVelCallback);
	ros::Subscriber re_vel_sub = nh.subscribe("read_right_elbow_velocity", 10, reVelCallback);

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
    // theta0 = theta0dot = 0; //gia dokimi

    // th0 = th1 = th2 = th3 = 0;  //arxikopoihsh kalou kakou
    ros::spinOnce();
    // calAngles();
    // updateVel(0.005); //200hz


    ROS_INFO("[Motors test]: You want to record to a bag? Press Y for yes, anything else for no. \n");

    std::cin>>cmd;
    if(cmd == 'Y'){
        record = true;
        ROS_INFO("[Motors test]: Please provide the name of the bag (dont put .bag). \n");
        std::cin >>  bag_file_name;
        bag.open(path + bag_file_name + ".bag", rosbag::bagmode::Write);
    }

     while(!offsetsdone){
        ROS_INFO("[Motors test]: Press Y to calculate angle offsets.");
        std::cin>>cmd;
        if(cmd == 'Y'){
            ros::spinOnce();
            loop_rate.sleep();
            offsetq1 = q1known - q1;
            offsetq2 = q2known - q2;
            offsetq3 = q3known - q3;
            ROS_INFO("Angle offsets have been calculated.");
            offsetsdone = true;

        }
        ros::Duration(2.0).sleep();
    }





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
        ros::spinOnce();
        std::cout<<"theta0 is: "<<theta0<<std::endl;
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
        // theta0 = q1 = q2 = q3 = q1dot = q2dot = q3dot = theta0dot = 0; //apla gia dokimi
        updateVel(0.005); //200hz
        // calcAngles();
        // if(count%100 == 0){
        //     std::cout<<"theta0 is: "<<(theta0*180/M_PI)<<" degrees."<<std::endl;
        //     std::cout<<"q1 is: "<<(q1*180/M_PI)<<" degrees."<<std::endl;
        //     std::cout<<"q2 is: "<<(q2*180/M_PI)<<" degrees."<<std::endl;
        //     std::cout<<"q3 is: "<<(q3*180/M_PI)<<" degrees."<<std::endl;
        // }
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
            // torq[0] = 25*errorq[0] + 5*errorqdot[0]; //apo emena afta
            // torq[1] = -(6*errorq[1] + 0.6*errorqdot[1])/186; //- giati ta mesa einai pros ta arnhtika , to /186 to vlepo pantou ston vraxiona
            // torq[2] = (6*errorq[2] + 0.6*errorqdot[2])/186;
            // torq[3] = -(6*errorq[3] + 0.6*errorqdot[3])/186;
            torq[0] = 0.5*errorq[0] + 2*errorqdot[0]; 
            torq[1] = 1.8*errorq[1] + 0.6*errorqdot[1]; 
            torq[2] = 1.7*errorq[2] + 0.4*errorqdot[2];
            torq[3] = 0.8*errorq[3] + 0.3*errorqdot[3];  //APO 0.3 KAI 0.2

            /*gia diabasma apo interface*/
            torq[1] = -torq[1]/186;
            torq[2] = torq[2]/186;
            torq[3] = -torq[3]/186;



            torque.data = filter_torque(torq[0], prev_torq[0]);
            rw_torque_pub.publish(torque);
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

            std::cout<<"RW torq  is: "<<torq[0]*186<< " Nm"<<std::endl;
            std::cout<<"q1 torq is: "<<-torq[1]*186<< " Nm"<<std::endl;
            std::cout<<"q2 torq is: "<<torq[2]*186<< " Nm"<<std::endl;
            std::cout<<"q3 torq is: "<<-torq[3]*186<< " Nm"<<std::endl;


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
        loop_rate.sleep();
    }

    if(record){
        bag.close();
    }

    return 0;
}