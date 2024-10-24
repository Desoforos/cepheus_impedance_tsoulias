/*
Nikiforos Tsoulias 2024
This will be the high level control node.
It shall read the state of the robot (joint posistions,force applied) and calculate the output wrench needed.
Then it shall publish it to the right topics:
In the real robot, it shall be the topics that the cepheus_interface reads.
*/

#include "includes.h"
#include "robot_variables.h"
#include "robot_callbacks.h"
#include "robot_functions.h"


#include <typeinfo>

// #define DESIRED_VEL 40  // RW_qdot_des [rad/s] //ta exo valei sta includes
// #define NUM_OF_MEASUREMENTS 1000
// #define POS_FILTER 0.005
// #define VEL_FILTER 0.05
// #define TORQUE_LIMIT 0.00000001

bool shutdown_requested = false;

void sigintHandler(int sig) {
    ROS_INFO("Shutdown request received. Performing cleanup tasks...");
    shutdown_requested = true;  // Set flag for graceful shutdown
}




int main(int argc, char **argv) {

    int count = 0;
    bool hasbegun = false;
    bool paramsinit = false;
    bool record = false;

    /* ros init */
    ros::init(argc, argv, "robot_foros_controller_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);

    ros::Time curr_time, t_beg;
    ros::Duration dur_time;
    double secs;
    // ros::Duration dur_time; //duration of movement
    //double dur_time;
    double tf; //time of movement before reaching target

    /* Create publishers */
  
    /*Publisher for debugging purposes*/
    ros::Publisher error_x_pub = nh.advertise<std_msgs::Float64>("/cepheus/error_x", 1);
    ros::Publisher error_y_pub = nh.advertise<std_msgs::Float64>("/cepheus/error_y", 1);
    ros::Publisher error_theta_pub = nh.advertise<std_msgs::Float64>("/cepheus/error_theta", 1);
    ros::Publisher xd_x_pub = nh.advertise<std_msgs::Float64>("/cepheus/xd_x", 1);
    ros::Publisher xd_y_pub = nh.advertise<std_msgs::Float64>("/cepheus/xd_y", 1);
    ros::Publisher xd_theta_pub = nh.advertise<std_msgs::Float64>("/cepheus/xd_theta", 1);
    ros::Publisher xt_x_pub = nh.advertise<std_msgs::Float64>("/cepheus/xt_x", 1);
    ros::Publisher xt_y_pub = nh.advertise<std_msgs::Float64>("/cepheus/xt_y", 1);
    ros::Publisher xt_theta_pub = nh.advertise<std_msgs::Float64>("/cepheus/xt_theta", 1);
    ros::Publisher xee_x_pub = nh.advertise<std_msgs::Float64>("/cepheus/xee_x", 1);
    ros::Publisher xee_y_pub = nh.advertise<std_msgs::Float64>("/cepheus/xee_y", 1);
    ros::Publisher xee_theta_pub = nh.advertise<std_msgs::Float64>("/cepheus/xee_theta", 1);



    ros::Publisher ls_torque_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_effort", 1);   //ropi se q1
	ros::Publisher le_torque_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_effort", 1);      //ropi se q2
  	ros::Publisher re_torque_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_effort", 1);     //ropi se q3 (ki omos to right elbow ousiastika einai to wrist)
    ros::Publisher rw_torque_pub = nh.advertise<std_msgs::Float64>("set_reaction_wheel_effort", 1);  //ropi se reaction wheel    (ola afta ta akouei to interface)
  


	ros::Publisher ls_offset_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_offset", 1);
	ros::Publisher le_offset_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_offset", 1);
    ros::Publisher re_offset_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_offset", 1);



    ros::Subscriber arduino_sub = nh.subscribe("/tsoulias_speak", 1, arduinoCallbacktest);
    ros::Publisher arduino_pub = nh.advertise<std_msgs::String>("/tsoulias_hear", 1);


	ros::Subscriber ls_pos_sub = nh.subscribe("read_left_shoulder_position", 1, lsPosCallback);  //gia q1
	ros::Subscriber le_pos_sub = nh.subscribe("read_left_elbow_position", 1, lePosCallback);     //gia q2
  	ros::Subscriber re_pos_sub = nh.subscribe("read_right_elbow_position", 1, rePosCallback);    //gia q3  
 	ros::Subscriber ls_vel_sub = nh.subscribe("read_left_shoulder_velocity", 1, lsVelCallback);  //gia q1dot   
    ros::Subscriber le_vel_sub = nh.subscribe("read_left_elbow_velocity", 1, leVelCallback);       //gia q2dot
   	ros::Subscriber re_vel_sub = nh.subscribe("read_right_elbow_velocity", 1, reVelCallback);    //gia q3dot

    // ros::Subscriber ls_limit_sub = nh.subscribe("read_left_shoulder_limit", 1, lsLimitCallback);  //afta einai gia arxikopoihsh , na ta do
	// ros::Subscriber le_limit_sub = nh.subscribe("read_left_elbow_limit", 1, leLimitCallback);

    ros::Subscriber force_sub = nh.subscribe("/filtered_botasys", 1, forceCallback);
    
    ros::Subscriber ee_pos_sub = nh.subscribe("/vicon/end_effector_new/end_effector_new", 1, ee_posCallback);
    ros::Subscriber target_pos_sub = nh.subscribe("/vicon/target_new/target_new", 1, target_posCallback);
    ros::Subscriber base_pos_sub = nh.subscribe("/vicon/base/base", 1, base_posCallback);


    /* init messages */ 
    msg_RW.data = 0.0;
    msg_LS.data = 0.0;
    msg_LE.data = 0.0;
    msg_LW.data = 0.0;

    msg_ex.data = 0.0;
    msg_ey.data = 0.0;
    msg_etheta.data = 0.0;
    // msg_TX.data = 0.0;
    // msg_TY.data = 0.0;

    ROS_INFO("[new_foros_simcontroller]: torques initialized to 0. \n");
    

    ros::Rate loop_rate(100); //100Hz

    char command;
    
    reachedTarget = false;
    start_movement = false;
    eefirstTime = true;
    targetfirstTime = true;
    basefirstTime = true;

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
        // ros::spinOnce(); //once it spins it will read the current rw, le, ls and the callbacks will update the values q1,q2,q3 and the velocities
        //now we update the errors and we recalculate the desired efforts to publish as msg_LE,msg_LS

        if(!start_movement){
            ROS_INFO("[new_foros_simcontroller]: Press Y to start the controller. \n");
            std::cin>> command;
            ros::spinOnce();
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
                initialiseParameters();
                ros::spinOnce();
                // ros::Duration(2).sleep();
                ROS_INFO("[new_foros_simcontroller]: Initializing parameters... \n");
                calculateTrajecotryPolynomials(tf);
                paramsinit = true;
                ROS_INFO("[new_foros_simcontroller]: Parameters have been initialized. \n");
                ROS_INFO("[new_foros_simcontroller]: Initializiing movement.");
            }
            /*MAIN CONTROL BODY*/
            ros::spinOnce();
            updateVel(0.01); // 100hz
            curr_time = ros::Time::now();
		    dur_time = curr_time - t_beg;
            secs = dur_time.sec + dur_time.nsec * pow(10, -9);
            finalTrajectories(secs,tf); //apo last_controller.h
            controller(count,tf,secs); //apo last_controller.h
            count++;
            if(incontact){
                contactCounter++;
            }
            else{
                contactCounter = 0;
            }
           if(contactCounter > 1.5*200){ // contact for 1.5sec
            beginGrab = true;
           }

           if(beginGrab){ 
            if(!beginSoft){
                beginSoft = true;
                ROS_INFO("[robot_foros_controller] Starting softgrip...");
                arduino_msg.data = "softgrip";
                arduino_pub.publish(arduino_msg);
                //ROS PUBLISH SOFTGRIP MIA FORA META DEN KSANASTELNEI
            }
            ros::spinOnce(); //to callback tou arduino tha kanei true to softFinished, an den doulevei apla perimeno 2 sec
            if(softFinished){
                if(!beginHard){
                    beginHard = true;
                    ROS_INFO("[robot_foros_controller] Softgrip ended! Starting hardgrip...");
                    arduino_msg.data = "hardgrip";
                    arduino_pub.publish(arduino_msg);
                    //ROSPUBLISH HARDGRIP MIA FORA META DEN KSANASTELNEI
                }
                ros::spinOnce(); //perimeno callback gia true to hardFinished
            }
           }
           if (hardFinished){
            ROS_INFO("Task completed. Ending now.");
            shutdown_requested = true;
           }

            if(!hardFinished){//EDO ZYNEXIZETAI NA STELNONTAI TIMES ROPON
                rw_torque_pub.publish(msg_RW);
                ls_torque_pub.publish(msg_LS);
                le_torque_pub.publish(msg_LE);
                re_torque_pub.publish(msg_LW); //ousiastika einai to left wrist alla tespa
            }
            msg_fextx.data = fext(0);
            xd_x_pub.publish(msg_xd_x);
            xd_y_pub.publish(msg_xd_y);
            xd_theta_pub.publish(msg_xd_theta);
            xt_x_pub.publish(msg_xt_x);
            xt_y_pub.publish(msg_xt_y);
            xt_theta_pub.publish(msg_xt_theta);
            xee_x_pub.publish(msg_xee_x);
            xee_y_pub.publish(msg_xee_y);
            xee_theta_pub.publish(msg_xee_theta);

            if(record){

                bag.write("/cepheus/xt_x", ros::Time::now(), msg_xt_x);
                bag.write("/cepheus/xd_x", ros::Time::now(), msg_xd_x);
                bag.write("/cepheus/xee_x", ros::Time::now(), msg_xee_x);

                bag.write("/cepheus/xt_y", ros::Time::now(), msg_xt_y);
                bag.write("/cepheus/xd_y", ros::Time::now(), msg_xd_y);
                bag.write("/cepheus/xee_y", ros::Time::now(), msg_xee_y);      

                bag.write("/cepheus/xt_theta", ros::Time::now(), msg_xt_theta);
                bag.write("/cepheus/xd_theta", ros::Time::now(), msg_xd_theta);
                bag.write("/cepheus/xee_theta", ros::Time::now(), msg_xee_theta);   

                bag.write("/cepheus/ft_sensor_topic", ros::Time::now(), msg_fextx);
               
            }
        }
        loop_rate.sleep();

    }

    if(shutdown_requested && record){
        bag.close();
    } 

    return 0;

}

