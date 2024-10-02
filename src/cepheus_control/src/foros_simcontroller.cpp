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

#include <typeinfo>

#define DESIRED_VEL 40  // RW_qdot_des [rad/s]
#define NUM_OF_MEASUREMENTS 1000
#define POS_FILTER 0.005
#define VEL_FILTER 0.05
#define TORQUE_LIMIT 0.00000001

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Matrix;

template<typename MatType>
using PseudoInverseType = Eigen::Matrix<typename MatType::Scalar, MatType::ColsAtCompileTime, MatType::RowsAtCompileTime>;

template<typename MatType>
PseudoInverseType<MatType> pseudoInverse(const MatType &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    using WorkingMatType = Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MatType::MaxRowsAtCompileTime, MatType::MaxColsAtCompileTime>;
    Eigen::BDCSVD<WorkingMatType> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(epsilon*std::max(a.cols(), a.rows()));
    Eigen::Index rank = svd.rank();
    Eigen::Matrix<typename MatType::Scalar, Eigen::Dynamic, MatType::RowsAtCompileTime,
    0, Eigen::BDCSVD<WorkingMatType>::MaxDiagSizeAtCompileTime, MatType::MaxRowsAtCompileTime>
    tmp = svd.matrixU().leftCols(rank).adjoint();
    tmp = svd.singularValues().head(rank).asDiagonal().inverse() * tmp;
    return svd.matrixV().leftCols(rank) * tmp;
}



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
    ros::init(argc, argv, "foros_simcontroller_node");
    ros::NodeHandle nh;
    ros::Time curr_time, t_beg;
    // ros::Duration dur_time; //duration of movement
    double dur_time;

    /* Create publishers */
    ros::Publisher RW_torque_pub = nh.advertise<std_msgs::Float64>("/cepheus/reaction_wheel_effort_controller/command", 1);
    ros::Publisher LS_torque_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_shoulder_effort_controller/command", 1);
    ros::Publisher LE_torque_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_elbow_effort_controller/command", 1);
    ros::Publisher LW_torque_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_wrist_effort_controller/command", 1);
    // ros::Publisher thruster_x_pub = nh.advertise<std_msgs::Float64>("/cepheus/thrusterx_effort_controller/command", 1); evgala ta dyo prismatic joints ki ebala gazebo plugin
    // ros::Publisher thruster_y_pub = nh.advertise<std_msgs::Float64>("/cepheus/thrustery_effort_controller/command", 1);
    ros::Publisher base_force_pub = nh.advertise<geometry_msgs::Wrench>("/cepheus/force_base_topic", 10);
    /*Publisher for debugging purposes*/
    ros::Publisher error_x_pub = nh.advertise<std_msgs::Float64>("/cepheus/error_x", 1);
    ros::Publisher error_y_pub = nh.advertise<std_msgs::Float64>("/cepheus/error_y", 1);
    ros::Publisher error_theta_pub = nh.advertise<std_msgs::Float64>("/cepheus/error_theta", 1);
    ros::Publisher xd_x_pub = nh.advertise<std_msgs::Float64>("/cepheus/xd_x", 1);
    ros::Publisher xd_y_pub = nh.advertise<std_msgs::Float64>("/cepheus/xd_y", 1);




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


    ROS_INFO("[foros_simcontroller]: torques initialized to 0. \n");
    

    // double frequency = (float)1/DT;

    /* Create subscribers */
    // ros::Subscriber RW_velocity_sub = nh.subscribe<sensor_msgs::JointState>("/cepheus/joint_states", 1, velocityCheckCallback);
    // ros::Subscriber position_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, positionCheckCallback);
	ros::Subscriber joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/cepheus/joint_states",1,jointStatesCallback);

    //ros::Subscriber ee_target_pos_sub = nh.subscribe<geometry_msgs::Pose>("/cepheus/ee_target_pos", 1, ee_target_posCallback);
    //ros::Subscriber ls_pos_sub = nh.subscribe("read_left_shoulder_position", 1, lsPosCallback);
    ros::Subscriber force_sub = nh.subscribe("/cepheus/ft_sensor_topic", 100, forceCallback);
    ros::Subscriber gazebo_pos_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states",1,gazeboposCallback);
    
    //ros::Rate loop_rate(frequency);
    ros::Rate loop_rate(10); //10Hz


	for (int i = 0; i < 3; i++) {  //initialize errors and torques
		errorq[i] = 0.0;
		error_qdot[i] = 0.0;
		torq[i] = 0.0;
		prev_torq[i] = 0.0;
	}

    
    // ROS_INFO("[foros_simcontroller]: Give me Kp, Kd. \n");
    // std::cin>>Kp>>Kd;  //kala einai ta kp=5 kai kd=0.5
    // ROS_INFO("[foros_simcontroller]: Give me q1des, q2des. \n");
    // std::cin>>q1des>>q2des; //apla gia na ksekinisei
    // q1des = q1des* (M_PI / 180);
    // q2des = q2des* (M_PI / 180);
    char command;
    //initialiseParameters();

    while(ros::ok()){
        // for (int i = 0; i < 3; i++) {
		// 	prev_torq[i] = torq[i];
		// }
        //ros::spinOnce(); //once it spins it will read the current rw, le, ls and the callbacks will update the values q1,q2,q3 and the velocities
        //now we update the errors and we recalculate the desired efforts to publish as msg_LE,msg_LS
        if(!start_movement){
            ROS_INFO("[foros_simcontroller]: Press Y to start the controller. Caution! Do not press it before running Gazebo. \n");
            std::cin>> command;
            if(command == 'Y'){
                start_movement= true;
            }
        }
        else{
            if(!hasbegun){
                ROS_INFO("[foros_simcontroller]: initializing movement");
                hasbegun = true; //apla gia to rosinfo na mas pei oti ksekinaei tin kinhsh
                t_beg  = ros::Time::now(); //initialize starting moment
            }
            if(!paramsinit){
                ros::spinOnce();
                // ros::Duration(2).sleep();
                ROS_INFO("[foros_simcontroller]: Initializing parameters... \n");
                initialiseParametersOLD();
                paramsinit = true;
                ROS_INFO("[foros_simcontroller]: Parameters have been initialized. \n");
                //continue;
                // ros::Duration(2).sleep();
            }
            ros::spinOnce();
            curr_time = ros::Time::now();
		    dur_time = curr_time.toSec() - t_beg.toSec();
            // if(dur_time<200){
            //     desiredTrajectory(dur_time);
            // } //mallon to xreiazetai tha to ksanavalo
            desiredTrajectory(dur_time);
            calculateMatrices();
            // diagnostics(); 

            //ImpedanceControlUpdateStep();

            /*UPDATE THE ROS MESSAGES*/
            // msg_TX.data = qact(0);
            // msg_TY.data = qact(1); 
            for(int i=0; i<5; i++){
                if(qact(i)>20){
                    qact(i) = 20;
                }
                else if(qact(i) < -20){
                     qact(i) = -20;
                }

            } //just to not send it to hell
            diagnostics(); 
            base_wrench.force.x = qact(0);//qact(0);
            base_wrench.force.y = qact(1);//qact(1);
            // base_wrench.torque.z = qact(2);
            msg_RW.data = qact(2); //to bazo anapoda bas kai
			msg_LS.data = qact(3);
			msg_LE.data = qact(4);
			msg_LW.data = qact(5);
            msg_ex.data = e(0); //for error plotting
            msg_ey.data = e(1);
            msg_etheta.data = e(2);
            msg_xd_x.data = xd(0);
            msg_xd_y.data = xd(1);


            // thruster_x_pub.publish(msg_TX);
            // thruster_y_pub.publish(msg_TY); den xreiazontai pia exo to base force


            base_force_pub.publish(base_wrench);
            RW_torque_pub.publish(msg_RW);
            LS_torque_pub.publish(msg_LS);
            LE_torque_pub.publish(msg_LE);
            LW_torque_pub.publish(msg_LW);
            error_x_pub.publish(msg_ex);
            error_y_pub.publish(msg_ey);
            error_theta_pub.publish(msg_etheta);
            xd_x_pub.publish(msg_xd_x);
            xd_y_pub.publish(msg_xd_y);



            //clear msgs after publish
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

              

        }
		if(reachedTarget){ //na ftiakso to reachedGoal kalytera gia na teleionei to peirama, na ftiakso xrono
			ROS_INFO("[foros_simcontroller]: target position achieved, stopped publishing. \n");
			break;
		}     
        //ros::spinOnce();
        //ros::Duration(2).sleep(); 
        loop_rate.sleep();

    }

    return 0;

}