/*
Nikiforos Tsoulias 2023
This will be the high level control node.
It shall read the state of the robot (joint posistions,force applied) and calculate the output wrench needed.
Then it shall publish it to the right topics:
In simulation, it shall be the gazebo command/effort topics
In the real robot, it shall be the topics that the cepheus_interface reads.
*/

#include "variables.h"
#include "callbacks.h"
#include "calculations.h"
#include "base_controller.h"
#include "last_controller.h"

#include <typeinfo>

#define DESIRED_VEL 40  // RW_qdot_des [rad/s]
#define NUM_OF_MEASUREMENTS 1000
#define POS_FILTER 0.005
#define VEL_FILTER 0.05
#define TORQUE_LIMIT 0.00000001

bool shutdown_requested = false;

void sigintHandler(int sig) {
    ROS_INFO("Shutdown request received. Performing cleanup tasks...");
    shutdown_requested = true;  // Set flag for graceful shutdown
}

double moving_average(double new_value, std::deque<double>& window, int size, double& running_sum) {
    if (window.size() == size) {
        running_sum -= window.front();
        window.pop_front();
    }
    window.push_back(new_value);
    running_sum += new_value;
    return running_sum / window.size();
}

std::deque<double> force_window;  // Stores the last N values
double sumforce;



/////////////// GLOBAL VARIABLES INITIALIZATION START////////////////////////

//einai sto variables.h

/////////////// GLOBAL VARIABLES INITIALIZATION END////////////////////////


/////////////// CALLBACK FUNCTIONS DEFINITION START////////////////////////

//einai sto callbacks.h

/////////////// CALLBACK FUNCTIONS DEFINITION END////////////////////////

/////////////// CALCULATION FUNCTIONS DEFINITION START////////////////////////

//einai sto calculations.h

/////////////// CALCULATION FUNCTIONS DEFINITION END////////////////////////

int main(int argc, char **argv) {

    int count = 0;
    bool hasbegun = false;
    bool paramsinit = false;
    bool record = false;
    

    double safetylimit = 25*M_PI/180;  //safetylimit 25 degrees to rad


    /* ros init */
    ros::init(argc, argv, "new_foros_simcontroller_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);

    ros::Time curr_time, t_beg;
    ros::Duration dur_time;
    double secs;
    // ros::Duration dur_time; //duration of movement
    //double dur_time;
    double tf; //time of movement before reaching target

    /* Create publishers */
    ros::Publisher RW_torque_pub = nh.advertise<std_msgs::Float64>("/cepheus/reaction_wheel_effort_controller/command", 100);
    ros::Publisher LS_torque_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_shoulder_effort_controller/command", 100);
    ros::Publisher LE_torque_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_elbow_effort_controller/command", 100);
    ros::Publisher LW_torque_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_wrist_effort_controller/command", 100);
    // ros::Publisher thruster_x_pub = nh.advertise<std_msgs::Float64>("/cepheus/thrusterx_effort_controller/command", 1); evgala ta dyo prismatic joints ki ebala gazebo plugin
    // ros::Publisher thruster_y_pub = nh.advertise<std_msgs::Float64>("/cepheus/thrustery_effort_controller/command", 1);
    ros::Publisher base_force_pub = nh.advertise<geometry_msgs::Wrench>("/cepheus/force_base_topic", 100); //anti gia 10 gia na doume
    /*Publisher for debugging purposes*/
    ros::Publisher error_x_pub = nh.advertise<std_msgs::Float64>("/cepheus/error_x", 100);
    ros::Publisher error_y_pub = nh.advertise<std_msgs::Float64>("/cepheus/error_y", 100);
    ros::Publisher error_theta_pub = nh.advertise<std_msgs::Float64>("/cepheus/error_theta", 100);
    ros::Publisher xd_x_pub = nh.advertise<std_msgs::Float64>("/cepheus/xd_x", 100);
    ros::Publisher xd_y_pub = nh.advertise<std_msgs::Float64>("/cepheus/xd_y", 100);
    ros::Publisher xd_theta_pub = nh.advertise<std_msgs::Float64>("/cepheus/xd_theta", 100);
    ros::Publisher xt_x_pub = nh.advertise<std_msgs::Float64>("/cepheus/xt_x", 100);
    ros::Publisher xt_y_pub = nh.advertise<std_msgs::Float64>("/cepheus/xt_y", 100);
    ros::Publisher xt_theta_pub = nh.advertise<std_msgs::Float64>("/cepheus/xt_theta", 100);
    ros::Publisher xee_x_pub = nh.advertise<std_msgs::Float64>("/cepheus/xee_x", 100);
    ros::Publisher xee_y_pub = nh.advertise<std_msgs::Float64>("/cepheus/xee_y", 100);
    ros::Publisher xee_theta_pub = nh.advertise<std_msgs::Float64>("/cepheus/xee_theta", 100);








    /* init messages */ 
    msg_RW.data = 0.0;
    msg_LS.data = 0.0;
    msg_LE.data = 0.0;
    msg_LW.data = 0.0;
    base_wrench.force.x = 0.0;
    base_wrench.force.y = 0.0;
    base_wrench.force.z = 0.0;
    base_wrench.torque.x = 0.0;
    base_wrench.torque.y = 0.0;
    base_wrench.torque.z = 0.0;
    msg_ex.data = 0.0;
    msg_ey.data = 0.0;
    msg_etheta.data = 0.0;
    // msg_TX.data = 0.0;
    // msg_TY.data = 0.0;


    ROS_INFO("[new_foros_simcontroller]: torques initialized to 0. \n");
    

    // double frequency = (float)1/DT;

    /* Create subscribers */
    // ros::Subscriber RW_velocity_sub = nh.subscribe<sensor_msgs::JointState>("/cepheus/joint_states", 1, velocityCheckCallback);
    // ros::Subscriber position_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, positionCheckCallback);
	
    ros::Subscriber joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/cepheus/joint_states",100,jointStatesCallback); //tha to anoikso

    //ros::Subscriber ee_target_pos_sub = nh.subscribe<geometry_msgs::Pose>("/cepheus/ee_target_pos", 1, ee_target_posCallback);
    //ros::Subscriber ls_pos_sub = nh.subscribe("read_left_shoulder_position", 1, lsPosCallback);
    ros::Subscriber force_sub = nh.subscribe("/cepheus/ft_sensor_topic", 100, forceCallback);
    
    ros::Subscriber gazebo_pos_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",100,gazeboposCallback);     //tha to anoikso
    
    //ros::Rate loop_rate(frequency);
    ros::Rate loop_rate(200); //200Hz
    double hz = 200; //200hz

    char command;
    
    wristInitialised = false; //true gia dokimi 
    
    reachedTarget = false;
    start_movement = false;
    firstTime = true;
    qfirstTime = true;
    safeclose = false;

    rosbag::Bag bag;
    std::string path = "/home/desoforos/cepheus_impedance_tsoulias/rosbags/" ;
    std::string bag_file_name;

    ROS_INFO("[new_foros_simcontroller]: You want to record to a bag? Press Y for yes, anything else for no. \n");

    std::cin>>command;
    if(command == 'Y'){
        record = true;
        ROS_INFO("[new_foros_simcontroller]: Please provide the name of the bag (dont put .bag). \n");
        std::cin >>  bag_file_name;
        bag.open(path + bag_file_name + ".bag", rosbag::bagmode::Write);
    }
    

    ROS_INFO("[new_foros_simcontroller]: Please provide the tf before proceeding. \n");

    std::cin>>tf;




    while(ros::ok() && !shutdown_requested){
        //ros::spinOnce(); //once it spins it will read the current rw, le, ls and the callbacks will update the values q1,q2,q3 and the velocities
        //now we update the errors and we recalculate the desired efforts to publish as msg_LE,msg_LS

        if(!start_movement){
            ROS_INFO("[new_foros_simcontroller]: Press Y to start the controller. Caution! Do not press it before running Gazebo. \n");
            std::cin>> command;
            ros::spinOnce();
            // loop_rate.sleep();
            xee_x_pub.publish(msg_xee_x);
            xee_y_pub.publish(msg_xee_y);
            xee_theta_pub.publish(msg_xee_theta);
            if(command == 'Y'){
                start_movement= true;
            }
        }
        else{
            if(!paramsinit){
                initialiseParametersNew();
                ros::spinOnce();
                // loop_rate.sleep();
                // ros::Duration(2).sleep();
                ROS_INFO("[new_foros_simcontroller]: Initializing parameters... \n");
                calculateTrajecotryPolynomials(tf);
                paramsinit = true;
                ROS_INFO("[new_foros_simcontroller]: Parameters have been initialized. \n");
                ROS_INFO("[new_foros_simcontroller]: Initializiing movement.");
                //continue;
                // ros::Duration(2).sleep();
            }
            ros::spinOnce();
            // loop_rate.sleep();
            curr_time = ros::Time::now();
		    dur_time = curr_time - t_beg;
            secs = dur_time.sec + dur_time.nsec * pow(10, -9);

            /*first initialise the wrist and keep q1,q2 steady*/
            if(false){ //!wristInitialised
                initialiseWrist();
            }
            else{
                if(!hasbegun){
                    ROS_INFO("[new_foros_simcontroller]: Initializiing control procedure.");
                    hasbegun = true; //apla gia to rosinfo na mas pei oti ksekinaei tin kinhsh
                    t_beg  = ros::Time::now(); //initialize starting moment
                }
                ros::spinOnce();
                // loop_rate.sleep();
                if(abs(q1)<safetylimit && abs(q2)<safetylimit){
                    if(!safeclose){
                        ROS_WARN("Arm extended! Initiating safe close...");
                        safeclose = true;
                        theta0fin = theta0;
                        theta0safeclose = theta0;
                        q1safeclose = q1;
                        q2safeclose = q2;
                        q3safeclose = q3;
                    }
                }
                if(!safeclose){ //1/200 = 0.005
                    fext(0) = abs(20*calcacc(0.005,xtdot,prevxtdot)); //override the plugin while experiment
                    fext(0) = moving_average(fext(0),force_window,10,sumforce);
                    curr_time = ros::Time::now();
                    dur_time = curr_time - t_beg;  //ksekinaei h metrhsh meta to initialization tou wrist, 30 sec gia wrist kai 30 sec gia ta alla
                    secs = dur_time.sec + dur_time.nsec * pow(10, -9);
                    finaltrajectories(secs,tf,hz); //apo last_controller.h
                    // diagnostics();
                    //desiredTrajectory(dur_time); 
                    // calculateMatrices(); //den xreiazetai pleon einai mesa sto controler()
                    // baseTrajectory(dur_time,tf);
                    // basePDcontroll();  //ena apo ta dyo tha exo anoikto
                    // calculateQ();
                    controller(count,tf,secs); //apo last_controller.h
                    count++;
                }
                else{
                    //safe close here
                    /*apla gia na kano ta plots meta kratao kai ta trajectories..*/
                    fext(0) = 0; //xanei epafh
                    curr_time = ros::Time::now();
                    dur_time = curr_time - t_beg;  //ksekinaei h metrhsh meta to initialization tou wrist, 30 sec gia wrist kai 30 sec gia ta alla
                    secs = dur_time.sec + dur_time.nsec * pow(10, -9);
                    finaltrajectories(secs,tf,hz); //apo last_controller.h

                    torqueRW = 0.06*(theta0safeclose - theta0) + 0.6*(0-theta0dot);
                    torqueq1 = 0.06*(q1safeclose-q1) + 0.6*(0-q1dot);
                    torqueq2 = 0.06*(q2safeclose-q2) + 0.6*(0-q2dot);
                    torqueq3 = 0.06*(q3safeclose-q3) + 0.6*(0-q3dot);
                    // torqueRW = torqueq1 = torqueq2 = torqueq3 = 0.0;
                }
                
            }

            base_wrench.torque.z = torqueRW;
            msg_LS.data = torqueq1;
            msg_LE.data = torqueq2;
            msg_LW.data = torqueq3;

            base_force_pub.publish(base_wrench);
            LS_torque_pub.publish(msg_LS);
            LE_torque_pub.publish(msg_LE);
            LW_torque_pub.publish(msg_LW);

            xd_x_pub.publish(msg_xd_x);
            xd_y_pub.publish(msg_xd_y);
            xd_theta_pub.publish(msg_xd_theta);
            xt_x_pub.publish(msg_xt_x);
            xt_y_pub.publish(msg_xt_y);
            xt_theta_pub.publish(msg_xt_theta);
            xee_x_pub.publish(msg_xee_x);
            xee_y_pub.publish(msg_xee_y);
            xee_theta_pub.publish(msg_xee_theta);
        }
        if(record){
                    msg_xd_x.data = xstep;
                    msg_xd_y.data = ystep;
                    msg_xd_theta.data = thstep;
                    msg_xd_theta0.data = theta0step;


                    msg_xt_x.data = xt;
                    msg_xt_y.data = yt;
                    msg_xt_theta.data = thetat;
                    msg_xt_theta0.data = theta0in;


                    msg_xee_x.data = xee(0);
                    msg_xee_y.data = xee(1);
                    msg_xee_theta.data = xee(2);
                    msg_xee_theta0.data = theta0;

                    msg_xd_x_dot.data = xstepdot;
                    msg_xd_y_dot.data = ystepdot;
                    msg_xd_theta_dot.data = thstepdot;
                    msg_xd_theta0_dot.data = theta0stepdot;

                    msg_xt_x_dot.data = xtdot;
                    msg_xt_y_dot.data = ytdot;
                    msg_xt_theta_dot.data = thetatdot;
                    msg_xt_theta0_dot.data = 0;

                    msg_xee_x_dot.data = xeedot(0);
                    msg_xee_y_dot.data = xeedot(1);
                    msg_xee_theta_dot.data = xeedot(2);
                    msg_xee_theta0_dot.data = theta0dot;

                    msg_fextx.data = fext(0);

                    msg_torquerw.data = torqueRW;
                    msg_torqueq1.data = torqueq1;
                    msg_torqueq2.data = torqueq2;
                    msg_torqueq3.data = torqueq3;

                    bag.write("/cepheus/xt_x", ros::Time::now(), msg_xt_x);
                    bag.write("/cepheus/xd_x", ros::Time::now(), msg_xd_x);
                    bag.write("/cepheus/xee_x", ros::Time::now(), msg_xee_x);


                    bag.write("/cepheus/xt_y", ros::Time::now(), msg_xt_y);
                    bag.write("/cepheus/xd_y", ros::Time::now(), msg_xd_y);
                    bag.write("/cepheus/xee_y", ros::Time::now(), msg_xee_y);      

                    bag.write("/cepheus/xt_theta", ros::Time::now(), msg_xt_theta);
                    bag.write("/cepheus/xd_theta", ros::Time::now(), msg_xd_theta);
                    bag.write("/cepheus/xee_theta", ros::Time::now(), msg_xee_theta);
                    
                    bag.write("/cepheus/xt_theta0", ros::Time::now(), msg_xt_theta0);
                    bag.write("/cepheus/xd_theta0", ros::Time::now(), msg_xd_theta0);
                    bag.write("/cepheus/xee_theta0", ros::Time::now(), msg_xee_theta0);     
                       

                    bag.write("/cepheus/ft_sensor_topic", ros::Time::now(), msg_fextx);


                    bag.write("/cepheus/xt_x_dot", ros::Time::now(), msg_xt_x_dot);
                    bag.write("/cepheus/xd_x_dot", ros::Time::now(), msg_xd_x_dot);
                    bag.write("/cepheus/xee_x_dot", ros::Time::now(), msg_xee_x_dot);

                    bag.write("/cepheus/xt_y_dot", ros::Time::now(), msg_xt_y_dot);
                    bag.write("/cepheus/xd_y_dot", ros::Time::now(), msg_xd_y_dot);
                    bag.write("/cepheus/xee_y_dot", ros::Time::now(), msg_xee_y_dot);      

                    bag.write("/cepheus/xt_theta_dot", ros::Time::now(), msg_xt_theta_dot);
                    bag.write("/cepheus/xd_theta_dot", ros::Time::now(), msg_xd_theta_dot);
                    bag.write("/cepheus/xee_theta_dot", ros::Time::now(), msg_xee_theta_dot);

                    bag.write("/cepheus/xt_theta0_dot", ros::Time::now(), msg_xt_theta0_dot);
                    bag.write("/cepheus/xd_theta0_dot", ros::Time::now(), msg_xd_theta0_dot);
                    bag.write("/cepheus/xee_theta0_dot", ros::Time::now(), msg_xee_theta0_dot);


                    bag.write("/cepheus/torquerw", ros::Time::now(), msg_torquerw);
                    bag.write("/cepheus/torqueq1", ros::Time::now(), msg_torqueq1);
                    bag.write("/cepheus/torqueq2", ros::Time::now(), msg_torqueq2);
                    bag.write("/cepheus/torqueq3", ros::Time::now(), msg_torqueq3);                  
                }

		if(reachedTarget){ //na ftiakso to reachedGoal kalytera gia na teleionei to peirama, na ftiakso xrono
			ROS_INFO("[new_foros_simcontroller]: Target position achieved, stopped publishing. \n");
			break;
		}   

        loop_rate.sleep(); //to exo balei allou

    }

     if(record && shutdown_requested){
            bag.close();
        }

    return 0;

}

