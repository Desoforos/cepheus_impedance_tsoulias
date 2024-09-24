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

    
    bool hasbegun = false;
    bool paramsinit = false;

    /* ros init */
    ros::init(argc, argv, "new_foros_simcontroller_node");
    ros::NodeHandle nh;
    ros::Time curr_time, t_beg;
    // ros::Duration dur_time; //duration of movement
    double dur_time;
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
	ros::Subscriber joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/cepheus/joint_states",100,jointStatesCallback);

    //ros::Subscriber ee_target_pos_sub = nh.subscribe<geometry_msgs::Pose>("/cepheus/ee_target_pos", 1, ee_target_posCallback);
    //ros::Subscriber ls_pos_sub = nh.subscribe("read_left_shoulder_position", 1, lsPosCallback);
    ros::Subscriber force_sub = nh.subscribe("/cepheus/ft_sensor_topic", 100, forceCallback);
    ros::Subscriber gazebo_pos_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",100,gazeboposCallback);
    
    //ros::Rate loop_rate(frequency);
    ros::Rate loop_rate(100); //100Hz

    char command;
    
    reachedTarget = false;
    start_movement = false;
    firstTime = true;

    ROS_INFO("[new_foros_simcontroller]: Please provide the tf before proceeding. \n");

    std::cin>>tf;




    while(ros::ok()){
        //ros::spinOnce(); //once it spins it will read the current rw, le, ls and the callbacks will update the values q1,q2,q3 and the velocities
        //now we update the errors and we recalculate the desired efforts to publish as msg_LE,msg_LS

        if(!start_movement){
            ROS_INFO("[new_foros_simcontroller]: Press Y to start the controller. Caution! Do not press it before running Gazebo. \n");
            std::cin>> command;
            ros::spinOnce();
            xee_x_pub.publish(msg_xee_x);
            xee_y_pub.publish(msg_xee_y);
            xee_theta_pub.publish(msg_xee_theta);
            if(command == 'Y'){
                start_movement= true;
            }
        }
        else{
            if(!hasbegun){
                ROS_INFO("[new_foros_simcontroller]: Initializiing control procedure.");
                hasbegun = true; //apla gia to rosinfo na mas pei oti ksekinaei tin kinhsh
                t_beg  = ros::Time::now(); //initialize starting moment
            }
            if(!paramsinit){
                ros::spinOnce();
                // ros::Duration(2).sleep();
                ROS_INFO("[new_foros_simcontroller]: Initializing parameters... \n");
                initialiseParameters();
                calculateTrajecotryPolynomials(tf);
                paramsinit = true;
                ROS_INFO("[new_foros_simcontroller]: Parameters have been initialized. \n");
                ROS_INFO("[new_foros_simcontroller]: Initializiing movement.");
                //continue;
                // ros::Duration(2).sleep();
            }
            ros::spinOnce();
            curr_time = ros::Time::now();
		    dur_time = curr_time.toSec() - t_beg.toSec();
 
            // diagnostics();
            //desiredTrajectory(dur_time); 
            // calculateMatrices(); //den xreiazetai pleon einai mesa sto controler()
            // baseTrajectory(dur_time,tf);
            finaltrajectories(dur_time); //apo last_controller.h
            // basePDcontroll();  //ena apo ta dyo tha exo anoikto
            // calculateQ();
            controller(); //apo last_controller.h


            // base_wrench.force.x = qact(0);  //fx;
            // base_wrench.force.y = qact(1);  //fy;
            // //base_wrench.torque.z = qact(2); //ns;
            // msg_RW.data = qact(2); //to bazo anapoda bas kai
			// msg_LS.data = qact(3);
			// msg_LE.data = qact(4);
			// msg_LW.data = qact(5);

            base_force_pub.publish(base_wrench);
            // RW_torque_pub.publish(msg_RW);
            LS_torque_pub.publish(msg_LS);
            LE_torque_pub.publish(msg_LE);
            LW_torque_pub.publish(msg_LW);

            // xd_x_pub.publish(msg_xd_x);
            // xd_y_pub.publish(msg_xd_y);
            // xd_theta_pub.publish(msg_xd_theta);
            // xt_x_pub.publish(msg_xt_x);
            // xt_y_pub.publish(msg_xt_y);
            // xt_theta_pub.publish(msg_xt_theta);
            // xee_x_pub.publish(msg_xee_x);
            // xee_y_pub.publish(msg_xee_y);
            // xee_theta_pub.publish(msg_xee_theta);

            base_wrench.force.x = 0.0;
            base_wrench.force.y = 0.0;
            base_wrench.force.z = 0.0;
            base_wrench.torque.x = 0.0;
            base_wrench.torque.y = 0.0;
            base_wrench.torque.z = 0.0;
            msg_RW.data = 0.0; //to bazo anapoda bas kai
	        msg_LS.data = 0.0;
	        msg_LE.data = 0.0;
	        msg_LW.data = 0.0; 
            msg_xt_x.data = 0.0;
            msg_xt_y.data = 0.0;
            msg_xt_theta.data = 0.0;
            msg_xd_x.data = msg_xd_y.data = msg_xd_theta.data = 0.0;

        }
		if(reachedTarget){ //na ftiakso to reachedGoal kalytera gia na teleionei to peirama, na ftiakso xrono
			ROS_INFO("[new_foros_simcontroller]: Target position achieved, stopped publishing. \n");
			break;
		}
    
        // ros::spinOnce();
        // std::cout<<"[Newforoscontroller callback] ee x is: "<<ee_x<<std::endl;
		// std::cout<<"[Newforoscontroller callback] ee y is: "<<ee_y<<std::endl;
		// std::cout<<"[Newforoscontroller callback] ee theta is: "<<thetach<<std::endl;
        // xee_x_pub.publish(msg_xee_x);
        // xee_y_pub.publish(msg_xee_y);
        // xee_theta_pub.publish(msg_xee_theta);

        loop_rate.sleep();

    }

    return 0;

}

//just to check