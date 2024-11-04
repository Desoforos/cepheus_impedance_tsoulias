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

double force_x;

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

void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr&msg){
    force_x = msg->wrench.force.x; //etsi einai mapped apo to botasys
    // std::cout<<"(forceCallback) I read: "<<force_X<<" N. "<<std::endl;
	if(abs(force_x)<0.25){
		incontact = false;
	}
	else{
		incontact = true;
	}
}



int main(int argc, char **argv) {


    /* ros init */
    ros::init(argc, argv, "arduino_test_node");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);

    ros::Subscriber arduino_sub = nh.subscribe("/tsoulias_speak", 1, arduinoCallbacktest);
    ros::Publisher arduino_pub = nh.advertise<std_msgs::String>("/tsoulias_hear", 1);

    ros::Subscriber force_sub = nh.subscribe("/filtered_botasys", 1, forceCallback);

    ros::Rate loop_rate(100); //100Hz

    int secs = 0; //not actually seconds
    int contactCounter = 0;

    
    char cmd;
    std::cout<<"Press any key to start."<<std::endl;
    std::cin>>cmd;


    while(ros::ok() && !shutdown_requested){
        if(incontact){
            contactCounter++;
            }
        else{
            contactCounter = 0;
            }
        if(contactCounter > 0.8*200){ // contact for 1.5sec
            beginGrab = true;
           }
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
            if(secs%100 == 0){
                std::cout<<"fext x is: "<<force_x<<" N. "<<std::endl;
            }
            secs++;
            ros::spinOnce();
            loop_rate.sleep();
    }


}
