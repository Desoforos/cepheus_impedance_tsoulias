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
// #include "IIRButterworthFilter.h"


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


class IIRButterworthFilter {
private:
    double a1, a2, b0, b1, b2;
    double x_prev1, x_prev2; // Previous input values (raw ydot)
    double y_prev1, y_prev2; // Previous output values (filtered ydot)

public:
    IIRButterworthFilter(double cutoff_freq, double sampling_freq) {
        double omega = 2.0 * M_PI * cutoff_freq / sampling_freq;
        double sin_omega = sin(omega);
        double cos_omega = cos(omega);
        double alpha = sin_omega / (2.0 * sqrt(2.0)); // Butterworth Q-factor

        // Butterworth filter coefficients (2nd-order)
        b0 = (1.0 - cos_omega) / 2.0;
        b1 = 1.0 - cos_omega;
        b2 = (1.0 - cos_omega) / 2.0;
        a1 = -2.0 * cos_omega;
        a2 = 1.0 - alpha;

        // Normalize coefficients
        double a0 = 1.0 + alpha;
        b0 /= a0;
        b1 /= a0;
        b2 /= a0;
        a1 /= a0;
        a2 /= a0;

        // Initialize previous values to zero
        x_prev1 = x_prev2 = 0.0;
        y_prev1 = y_prev2 = 0.0;
    }

    double filter(double x) {
        // Apply the IIR difference equation
        double y = b0 * x + b1 * x_prev1 + b2 * x_prev2 - a1 * y_prev1 - a2 * y_prev2;

        // Update history for next iteration
        x_prev2 = x_prev1;
        x_prev1 = x;
        y_prev2 = y_prev1;
        y_prev1 = y;

        return y;
    }
};





int main(int argc, char **argv) {

    // ee_x = -0.5;
    // ee_y = 0.175;
    // thetach = -70*M_PI/180;

    // xeedot(0) = xeedot(1) = xeedot(2) = 0;
    // theta0 = 0;
    // theta0dot = 0;
    // xt = 0;
    // yt = 0;
    // thetat = 0;
    // xtdot = 0;
    // ytdot = 0;
    // thetatdot = 0;


    double safetylimit = 20*M_PI/180; 
    int count = 0;
    bool hasbegun = false;
    bool paramsinit = false;
    bool record = false;
    bool grabStarted = false;

    targetcheck = false;
    eecheck = false;
    basecheck = false;

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

    // IIRButterworthFilter ydot_filter(0.8, 100.0);

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
    ros::Publisher rw_torque_pub = nh.advertise<std_msgs::Float64>("cmd_torque", 1);  //ropi se reaction wheel    (ola afta ta akouei to interface)
  


	ros::Publisher ls_offset_pub = nh.advertise<std_msgs::Float64>("set_left_shoulder_offset", 1);
	ros::Publisher le_offset_pub = nh.advertise<std_msgs::Float64>("set_left_elbow_offset", 1);
    ros::Publisher re_offset_pub = nh.advertise<std_msgs::Float64>("set_right_elbow_offset", 1);
    ros::Publisher start_moving_pub = nh.advertise<std_msgs::Bool>("start_moving",1) ;//na dosei entolh ksereis kati ksekinao peirama

    ros::Publisher grab_pub =  nh.advertise<std_msgs::Bool>("start_grab",1);



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
    
    ros::Subscriber ee_pos_sub = nh.subscribe("/vicon/cepheus_endeffector/cepheus_endeffector", 1, ee_posCallback);
    ros::Subscriber target_pos_sub = nh.subscribe("/vicon/cepheus_target/cepheus_target", 1, target_posCallback);
    ros::Subscriber base_pos_sub = nh.subscribe("/vicon/cepheusbase/cepheusbase", 1, base_posCallback);


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

    tau(0) = tau(1) = tau(2) = tau(3) = 0.0001;

    prev_tau(0) = 0.0001;
    prev_tau(1) = 0.0001;
    prev_tau(2) = 0.0001;
    prev_tau(3) = 0.0001;

    ROS_INFO("[new_foros_simcontroller]: Prev and current torques initialized to 0. \n");
    

    ros::Rate loop_rate(100); //100Hz

    char command;
    char cmd;
    
    reachedTarget = false;
    start_movement = false;
    // eefirstTime = true;
    // targetfirstTime = true;
    // targetfirstTime = false; //apla gia test
    // basefirstTime = true;
    firstTime = true;

    rosbag::Bag bag;
    std::string path = "/home/desoforos/cepheus_impedance_tsoulias/rosbags/" ;
    std::string bag_file_name;

    while(!offsetsdone){
        ROS_INFO("[robot_foros_controller]: Press Y to calculate angle offsets.");
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

    ROS_INFO("[robot_foros_simcontroller]: You want to record to a bag? Press Y for yes, anything else for no. \n");
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
            loop_rate.sleep();
            if(command == 'Y'){
                start_movement= true;
                // start_moving.data = true;
                // start_moving_pub.publish(start_moving);
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
                // while(!(targetcheck && eecheck && basecheck)){
                //     ros::spinOnce();
                //     loop_rate.sleep();
                // } // na sigourefto oti exei akousei tin vicon gia ola ta frames
                // ros::Duration(2).sleep();
                ROS_INFO("[new_foros_simcontroller]: Initializing parameters... \n");
                calculateTrajecotryPolynomials(tf);
                paramsinit = true;
                ROS_INFO("[new_foros_simcontroller]: Parameters have been initialized. \n");
                ROS_INFO("[new_foros_simcontroller]: Initializiing movement.");
            }
            /*MAIN CONTROL BODY*/
            ros::spinOnce();
            // loop_rate.sleep();
            /*control function takes place here, PD/impedance etc*/
            curr_time = ros::Time::now();
            dur_time = curr_time - t_beg;
            secs = dur_time.sec + dur_time.nsec * pow(10, -9);
            // if(secs>=tf){ //to bazo gia na kanei douleia mexri ta 30, tha to bgalo afto meta
            //     msg_RW.data = msg_LS.data = msg_LE.data = msg_LW.data =  0.0001;
            //     rw_torque_pub.publish(msg_RW);
            //     ls_torque_pub.publish(msg_LS);
            //     le_torque_pub.publish(msg_LE);
            //     re_torque_pub.publish(msg_LW);
            //     shutdown_requested = true;
            //     break;
            // }
            updateVel(0.01,secs,tf); // 100hz
            // xeedot(1) = ydot_filter.filter(xeedot(1));
            finalTrajectories(secs,tf); //gia polyonymikh troxia, tin theloume gia olous tous controllers, kanei kai arxikopoihsh metavlhton
            controller(count,tf,secs); //impedance controller
            // ee_x = xstep - 0.001;
            // ee_y = ystep - 0.001;
            // thetach = thstep - 1*M_PI/180;

            // xeedot(0) = xstepdot - 0.01;
            // xeedot(1) = ystepdot - 0.01;
            // xeedot(2) = thstepdot - 1*M_PI/180;
            // PDcontroller(tf,secs); //cartesian PD controller
            count++;
            msg_RW.data = filter_torque(tau(0),prev_tau(0)); //tau(0); 
            msg_LS.data = filter_torque(tau(1),prev_tau(1)); //tau(1);
            msg_LE.data = filter_torque(tau(2),prev_tau(2)); //tau(2);
            msg_LW.data = filter_torque(tau(3),prev_tau(3));
            rw_torque_pub.publish(msg_RW);
            ls_torque_pub.publish(msg_LS);
            le_torque_pub.publish(msg_LE);
            re_torque_pub.publish(msg_LW); //ousiastika einai to left wrist alla tespa
            if(secs>tf && (abs(ee_x-xt)<0.003) && (abs(ee_y-yt)<0.003) && !grabStarted){
                grabStarted = true;
                start_grab_msg.data = true;
                grab_pub.publish(start_grab_msg);
            }
            if(record){

                // if(safeclose){
                //     xstep = xsafeclose;
                //     ystep = ysafeclose;
                //     thstep = thetasafeclose;
                //     xstepdot = 0;
                //     ystepdot = 0;
                //     thstepdot = 0;
                //     theta0step = theta0safeclose;
                //     theta0stepdot = 0;

                // }

                msg_xd_x.data = xstep;
                msg_xd_y.data = ystep;
                msg_xd_theta.data = thstep;
                msg_xd_theta0.data = theta0step;


                msg_xt_x.data = xt;
                msg_xt_y.data = yt;
                msg_xt_theta.data = thetat;

                // msg_xt_x_raw.data = rawxt;
                // msg_xt_y_raw.data = rawyt;
                // msg_xt_theta_raw.data = rawthetat;

                msg_xt_theta0.data = theta0in;


                msg_xee_x.data = ee_x;
                msg_xee_y.data = ee_y;
                msg_xee_theta.data = thetach;
                msg_xee_theta0.data = theta0;

                msg_xd_x_dot.data = xstepdot;
                msg_xd_y_dot.data = ystepdot;
                msg_xd_theta_dot.data = thstepdot;
                msg_xd_theta0_dot.data = theta0stepdot;


                msg_xt_x_dot.data = xtdot;
                msg_xt_y_dot.data = ytdot;
                msg_xt_theta_dot.data = thetatdot;

                msg_xt_x_dot_raw.data = rawxtdot;
                msg_xt_y_dot_raw.data = rawytdot;
                msg_xt_theta_dot_raw.data = rawthetatdot;

                msg_xt_theta0_dot.data = 0;

                msg_xee_x_dot.data = xeedot(0);
                msg_xee_y_dot.data = xeedot(1);
                msg_xee_theta_dot.data = xeedot(2);
                msg_xee_theta0_dot.data = theta0dot;

                msg_fextx.data = force_x;
                msg_fextx_raw.data = raw_force_x;

                msg_q1.data = q1;
                msg_q2.data = q2;
                msg_q3.data = q3;


                msg_q1dot.data = q1dot;
                msg_q2dot.data = q2dot;
                msg_q3dot.data = q3dot;

                    
                msg_torquerw.data = tau(0);
                msg_torqueq1.data = tau(1);
                msg_torqueq2.data = tau(2);
                msg_torqueq3.data = tau(3);   

                // if(count <1){  //giati to xstep[0] einai 0
                //     // std::cout<<"frist torques zero"<<std::endl;
                //     msg_RW.data = msg_LS.data = msg_LE.data = msg_LW.data =  0.0;
                // }

                // bag.write("/cepheus/xt_x_raw", ros::Time::now(), msg_xt_x_raw);
                // bag.write("/cepheus/xt_y_raw", ros::Time::now(), msg_xt_y_raw);
                // bag.write("/cepheus/xt_theta_raw", ros::Time::now(), msg_xt_theta_raw);

                // bag.write("/cepheus/xt_x_dot_raw", ros::Time::now(), msg_xt_x_dot_raw);
                // bag.write("/cepheus/xt_y_dot_raw", ros::Time::now(), msg_xt_y_dot_raw);
                // bag.write("/cepheus/xt_theta_dot_raw", ros::Time::now(), msg_xt_theta_dot_raw);


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

                bag.write("/cepheus/ft_sensor_topic", ros::Time::now(), msg_fextx);
                bag.write("/cepheus/fextx_raw", ros::Time::now(), msg_fextx_raw);
            
                bag.write("/cepheus/torquerw", ros::Time::now(), msg_torquerw);
                bag.write("/cepheus/torqueq1", ros::Time::now(), msg_torqueq1);
                bag.write("/cepheus/torqueq2", ros::Time::now(), msg_torqueq2);
                bag.write("/cepheus/torqueq3", ros::Time::now(), msg_torqueq3); 

                bag.write("/cepheus/q1", ros::Time::now(), msg_q1);
                bag.write("/cepheus/q2", ros::Time::now(), msg_q2);
                bag.write("/cepheus/q3", ros::Time::now(), msg_q3);
    

                bag.write("/cepheus/q1dot", ros::Time::now(), msg_q1dot);
                bag.write("/cepheus/q2dot", ros::Time::now(), msg_q2dot);
                bag.write("/cepheus/q3dot", ros::Time::now(), msg_q3dot);
    
                
                }
        }
            
            loop_rate.sleep();  //exei bei allou, vasika kalytera edo
    }
    

    if(shutdown_requested && record){
        bag.close();
    } 

    // start_moving.data = false;
    // start_moving_pub.publish(start_moving);


    return 0;

}

/*
if(incontact){
                contactCounter++;
            }
            else{
                contactCounter = 0;
            }
           if(contactCounter > 1*200){ // contact for 1 sec
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
            // loop_rate.sleep();
            if(softFinished){
                if(!beginHard){
                    beginHard = true;
                    ROS_INFO("[robot_foros_controller] Softgrip ended! Starting hardgrip...");
                    arduino_msg.data = "hardgrip";
                    arduino_pub.publish(arduino_msg);
                    //ROSPUBLISH HARDGRIP MIA FORA META DEN KSANASTELNEI
                }
                ros::spinOnce(); //perimeno callback gia true to hardFinished
                // loop_rate.sleep();
            }
           }
        if (hardFinished){
            ROS_INFO("Task completed. Ending now.");
            // start_moving.data = false; //den to xrhsimopoio telika
            // start_moving_pub.publish(start_moving);
            shutdown_requested = true;
            ROS_INFO("[robot_foros_controller] Hardgrip ended! Starting safeclose...");
            // safeclose = true; //to sbino gia ligo
            theta0fin = theta0;
            theta0safeclose = theta0;
            q1safeclose = q1; //gia na meinei akinhto
            q2safeclose = q2;
            q3safeclose = q3;
            xsafeclose = ee_x;
            ysafeclose = ee_y;
            thetasafeclose = thetach;
           }
        if(safeclose){//edo ginetai overwrite timon gia to safeclose            
            tau(0) = 0.5*(theta0safeclose - theta0) + 2*(0-theta0dot);
            tau(1) = 1.3*(q1safeclose-q1) + 0.6*(0-q1dot);
            tau(2) = 1.3*(q2safeclose-q2) + 0.6*(0-q2dot);
            tau(3) = 0.8*(q3safeclose-q3) + 0.6*(0-q3dot);

            tau(1) = -tau(1)/186;
            tau(2) = tau(2)/186;
            tau(3) = -tau(3)/186;
            msg_LS.data = filter_torque(tau(1),prev_tau(1)); //tau(1);
            msg_LE.data = filter_torque(tau(2),prev_tau(2)); //tau(2);
            msg_LW.data = filter_torque(tau(3),prev_tau(3));
            rw_torque_pub.publish(msg_RW);
            ls_torque_pub.publish(msg_LS);
            le_torque_pub.publish(msg_LE);
            re_torque_pub.publish(msg_LW); //ousiastika einai to left wrist alla tespa
        }
*/