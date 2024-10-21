#include "includes.h"
#include <typeinfo>

bool beginGrab = false;
bool incontact =false;
bool beginSoft = false;
bool softFinished = false;
bool beginHard = false;
bool hardFinished = false;

bool gripperListenedSoft = false;
bool gripperListenedHard = false;

bool shutdown_requested = false;

std_msgs::String arduino_msg; 

void sigintHandler(int sig) {
    ROS_INFO("Shutdown request received. Performing cleanup tasks...");
    shutdown_requested = true;  // Set flag for graceful shutdown
}

void arduinoCallbacktest(const std_msgs::String::ConstPtr &msg){
	if(msg->data == "nothing"){
		if(gripperListenedSoft){
			softFinished = true;
		}
		if(gripperListenedHard){
			hardFinished = true;
		}
	}
	if(msg->data == "softgrip"){
		gripperListenedSoft = true;
	}
	if(msg->data == "hardgrip"){
		gripperListenedHard = true;
	}
}



int main(int argc, char **argv) {


    /* ros init */
    ros::init(argc, argv, "arduino_test_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);

    ros::Subscriber arduino_sub = nh.subscribe("/tsoulias_speak", 1, arduinoCallbacktest);
    ros::Publisher arduino_pub = nh.advertise<std_msgs::String>("/tsoulias_hear", 1);

    
    char cmd;
    std::cout<<"if you want to begin grab press Y"<<std::endl;
    std::cin>>cmd;
    if(cmd == 'Y') beginGrab=true;
    cmd = 'N';

    while(ros::ok() && !shutdown_requested){
        if(beginGrab){ 
                if(!beginSoft){
                    beginSoft = true;
                    std::cout<<"Starting softgrip..."<<std::endl;
                    arduino_msg.data = "softgrip";
                    arduino_pub.publish(arduino_msg);
                    //ROS PUBLISH SOFTGRIP MIA FORA META DEN KSANASTELNEI

                }
                ros::spinOnce(); //to callback tou arduino tha kanei true to softFinished, an den doulevei apla perimeno 2 sec
                if(softFinished){
                    if(!beginHard){
                        beginHard = true;
                        std::cout<<"Softgrip ended! Starting hardgrip..."<<std::endl;
                        // std::cin>>cmd;
                        // if(cmd == 'Y'){
                        //     std::cout<<"Starting hardgrip..."<<std::endl;
                        // }
                        arduino_msg.data = "hardgrip";
                        arduino_pub.publish(arduino_msg);
                        //ROSPUBLISH HARDGRIP MIA FORA META DEN KSANASTELNEI
                    }
                    ros::spinOnce(); //perimeno callback gia true to hardFinished
                }
            }
            if (hardFinished){
                std::cout<<"Hardgrip finished! Press any key to release."<<std::endl;
                std::cin>>cmd;
                arduino_msg.data = "release";
                arduino_pub.publish(arduino_msg);
                shutdown_requested = true;
            }
    }

}
