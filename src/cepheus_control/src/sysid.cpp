/*einai akoma prototype, exo valei ton kormo omos pistevo den thelei poly akoma*/
#include "includes.h"
#include "robot_variables.h"
#include "robot_callbacks.h"
#include "robot_functions.h"

#include <typeinfo>

bool shutdown_requested = false;

void sigintHandler(int sig) {
    ROS_INFO("Shutdown request received. Performing cleanup tasks...");
    shutdown_requested = true;  // Set flag for graceful shutdown
}

int main(int argc, char **argv){
    
    geometry_msgs::Vector3Stamped thrust_vector;

    /* ros init */
    ros::init(argc, argv, "sysid_node");
    ros::NodeHandle nh;
    ros::Publisher thrust_pub = nh.advertise<geometry_msgs::Vector3Stamped>("cmd_thrust", 1);
    signal(SIGINT, sigintHandler);

    ros::Time curr_time, t_beg;
    ros::Duration dur_time;
    double secs;
    ros::Rate loop_rate(100);

    thrust_vector.vector.x = 0.0;
	thrust_vector.vector.y = 0.0;
	thrust_vector.vector.z = 0.0;
	thrust_pub.publish(thrust_vector);
    int counter = 0;
    bool positive = true;

    while(!shutdown_requested){
        if(counter%10 ==0){  //10 = 0.1*100Hz
            positive = !positive;
        }
        if(positive){
            thrust_vector.vector.z = 0.1;
        }
        else{
            thrust_vector.vector.z = -0.1;
        }
        counter++;
        thrust_pub.publish(thrust_vector);
    }

    return 0;
}
