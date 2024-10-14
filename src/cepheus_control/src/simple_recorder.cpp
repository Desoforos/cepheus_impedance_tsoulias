#include "includes.h"

double velprev =0;
double velcurr =0;
double acc =0;
double torque =0;
bool shutdown_requested = false;
double deltatime;
double prevtime = 0;
double currtime = 0;
double velcurrimu =0;
double velprevimu =0;
double accimu =0;
double velcurrjoint =0;
double velprevjoint =0;
double accjoint =0;
int count =0;


void sigintHandler(int sig) {
    ROS_INFO("Shutdown request received. Performing cleanup tasks...");
    shutdown_requested = true;  // Set flag for graceful shutdown
}

void gazeboposCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){
    for(int i=0; i<msg->name.size(); i++){
		if(msg->name[i] == "cepheus::final_link"){
            velprev = velcurr;
			velcurr= msg->twist[i].angular.z;
		}
    }
    // if(count%100 == 0){
    //     std::cout<<"Time is: "<<ros::Time::now() <<std::endl;
    //     std::cout<<"current vel is: "<<velcurr<<std::endl;
    //     std::cout<<"//////END/////////"<<std::endl;
    // }
}

void torqueCallback(const std_msgs::Float64::ConstPtr& msg){
    torque = msg->data;
}


void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){
	int i;
    //ROS_INFO("[foros_simcontroller]: Joint state received! q1: %f q2: %f q1dot: %f q2dot: %f \n",msg->position[1], msg->position[0], msg-> velocity[1], msg-> velocity[0]);
	for(i=0; i<msg->name.size(); i++){
		// ROS_INFO("[Gazebo Callback] Joint Name: %s", msg->name[i]);
        if(msg->name[i] == "left_wrist_joint"){
            velprevjoint = velcurrjoint;
			velcurrjoint = msg->velocity[i];
		}
	}
}


void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    velprevimu = velcurrimu;
    velcurrimu = msg->angular_velocity.z; 
}



int main(int argc, char **argv) {

    /* ros init */
    ros::init(argc, argv, "simple_recorder_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);
    ros::Rate loop_rate(100); //100hz
    double dt = 0.01;

    ros::Subscriber gazebo_pos_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",100,gazeboposCallback);
    ros::Subscriber LW_torque_sub = nh.subscribe<std_msgs::Float64>("/cepheus/left_wrist_effort_controller/command",100,torqueCallback);
    ros::Subscriber joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/cepheus/joint_states",100,jointStatesCallback);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/cepheus/imu",100,imuCallback);


    rosbag::Bag bag;
    std::string path = "/home/desoforos/cepheus_impedance_tsoulias/rosbags/" ;
    std::string bag_file_name;
    std_msgs::Float64 msg_gazebo;
    std_msgs::Float64 msg_torque;
    std_msgs::Float64 msg_imu;
    std_msgs::Float64 msg_joint;





    ROS_INFO("[new_foros_simcontroller]: Please provide the name of the bag (dont put .bag). \n");
    std::cin >>  bag_file_name;
    bag.open(path + bag_file_name + ".bag", rosbag::bagmode::Write);

    ros::Time prev_time = ros::Time::now();

    

    ROS_INFO("Starting recording of  ee_thetadotdot and torque of ee.");
    while(ros::ok() && !shutdown_requested){
        ros::spinOnce();
        count++ ;
        ros::Time curr_time = ros::Time::now();
        ros::Duration deltaTime = curr_time - prev_time;
        prev_time = curr_time;

        // acc = (velcurr-velprev)/deltaTime.toSec();
        // accimu = (velcurrimu-velprevimu)/deltaTime.toSec();
        // accjoint = (velcurrjoint-velprevjoint)/deltaTime.toSec();

        acc = (velcurr-velprev)/dt;
        accimu = (velcurrimu-velprevimu)/dt;
        accjoint = (velcurrjoint-velprevjoint)/dt;

        // msg_gazebo.data = acc;
        // msg_imu.data = accimu;
        // msg_joint.data = accjoint;

        msg_gazebo.data = velcurr;
        msg_imu.data = velcurrimu;
        msg_joint.data = velcurrjoint;

        msg_torque.data = torque;

        if(count%100 == 0){
            std::cout<<"Time is: "<<ros::Time::now() <<std::endl;
            std::cout<<"current imu vel is: "<<velcurrimu<<std::endl;
            std::cout<<"//////END/////////"<<std::endl;
        }


        bag.write("/cepheus/left_wrist_torque", ros::Time::now(), msg_torque);
        bag.write("/cepheus/accgazebo", ros::Time::now(), msg_gazebo);
        bag.write("/cepheus/accimu", ros::Time::now(), msg_imu);
        bag.write("/cepheus/accjoint", ros::Time::now(), msg_joint);



            
        loop_rate.sleep();
    }
    if(shutdown_requested){
        bag.close();
    }
        

    return 0;
}
