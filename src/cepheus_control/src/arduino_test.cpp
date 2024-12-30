#include "includes.h"
#include <typeinfo>

bool beginGrab = false;
bool ardincontact =false;
bool beginSoft = false;
bool softFinished = false;
bool beginHard = false;
bool hardFinished = false;

bool gripperListenedSoft = false;
bool gripperListenedHard = false;

bool shutdown_requested = false;

double force_x, raw_force_x;

std_msgs::String arduino_msg; 

double forcesum = 0;
int force_window_size = 10;
std::deque<double> force_window; 

double moving_average(double new_value, std::deque<double>& window, int size, double& running_sum) {
    if (window.size() == size) {
        running_sum -= window.front();
        window.pop_front();
    }
    window.push_back(new_value);
    running_sum += new_value;
    return running_sum / window.size();
}


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
    raw_force_x = abs(msg->wrench.force.z); //etsi einai mapped apo to botasys
    // std::cout<<"(forceCallback) I read: "<<force_X<<" N. "<<std::endl;
    // force_x = moving_average(raw_force_x, force_window, force_window_size, forcesum);
	if(abs(raw_force_x)<0.4){
		ardincontact = false;
	}
	else{
		ardincontact = true;
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

    ros::Rate loop_rate(200); //200Hz
    std_msgs::Float64 force_msg;
    std_msgs::Float64 raw_force_msg;

    bool record = false;

    int secs = 0; //not actually seconds
    int contactCounter = 0;
    char cmd;
    rosbag::Bag bag;
    std::string path = "/home/desoforos/cepheus_impedance_tsoulias/rosbags/" ;
    std::string bag_file_name;

    ROS_INFO("[Arduino test]: You want to record to a bag? Press Y for yes, anything else for no. \n");

    std::cin>>cmd;
    if(cmd == 'Y'){
        record = true;
        ROS_INFO("[Motors test]: Please provide the name of the bag (dont put .bag). \n");
        std::cin >>  bag_file_name;
        bag.open(path + bag_file_name + ".bag", rosbag::bagmode::Write);
    }

    std::cout<<"Press any key to start."<<std::endl;
    std::cin>>cmd;


    while(ros::ok() && !shutdown_requested){
        if(ardincontact){
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

            if(record){
                force_msg.data = force_x;
                raw_force_msg.data = raw_force_x;
                bag.write("/cepheus/raw_force_x", ros::Time::now(), raw_force_msg);
                bag.write("/cepheus/force_x", ros::Time::now(), force_msg);
            }
            ros::spinOnce();
            loop_rate.sleep();
    }
    if(record){
        bag.close();
    }


    return 0;
}
