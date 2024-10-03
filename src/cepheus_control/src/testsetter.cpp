#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include "includes.h"
#include "variables.h"


bool switchControllers(ros::NodeHandle& nh, 
                       const std::vector<std::string>& stop_controllers, 
                       const std::vector<std::string>& start_controllers) {
    ros::ServiceClient switch_client = nh.serviceClient<controller_manager_msgs::SwitchController>("/cepheus/controller_manager/switch_controller");

    controller_manager_msgs::SwitchController switch_srv;
    switch_srv.request.stop_controllers = stop_controllers;
    switch_srv.request.start_controllers = start_controllers;
    switch_srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

    if (switch_client.call(switch_srv)) {
        return switch_srv.response.ok;
    } else {
        ROS_ERROR("Failed to call service switch_controller");
        return false;
    }
}

bool loadController(ros::NodeHandle& nh, const std::string& controller_name) {
    ros::ServiceClient load_client = nh.serviceClient<controller_manager_msgs::LoadController>("/cepheus/controller_manager/load_controller");

    controller_manager_msgs::LoadController load_srv;
    load_srv.request.name = controller_name;

    if (load_client.call(load_srv)) {
        return load_srv.response.ok;
    } else {
        ROS_ERROR("Failed to call service load_controller");
        return false;
    }
}

bool unloadController(ros::NodeHandle& nh, const std::string& controller_name) {
    ros::ServiceClient unload_client = nh.serviceClient<controller_manager_msgs::UnloadController>("/cepheus/controller_manager/unload_controller");

    controller_manager_msgs::UnloadController unload_srv;
    unload_srv.request.name = controller_name;

    if (unload_client.call(unload_srv)) {
        return unload_srv.response.ok;
    } else {
        ROS_ERROR("Failed to call service unload_controller");
        return false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "testsetter_node");
    ros::NodeHandle nh;

    std::vector<std::string> controllersA = {"left_shoulder_position_controller", "left_elbow_position_controller", "left_wrist_position_controller"};
    std::vector<std::string> controllersB = {"left_shoulder_effort_controller", "left_elbow_effort_controller", "left_wrist_effort_controller"};

    /* Create publishers */
    ros::Publisher LS_position_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_shoulder_position_controller/command", 1);
    ros::Publisher LE_position_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_elbow_position_controller/command", 1);
    ros::Publisher LW_position_pub = nh.advertise<std_msgs::Float64>("/cepheus/left_wrist_position_controller/command", 1);

    /* messages to publish */
    std_msgs::Float64 msg_LS;
    std_msgs::Float64 msg_LE;
    std_msgs::Float64 msg_LW;

    // double q1,q2,q3,q01;
    // double xE_in,yE_in,thetaE_in;
    // double xb_in,yb_in,thetab_in;
    // double cos_q2, sin_q2, sint_th0_q1, cos_th0_q1;
    // double q1_in, q2_in,q3_in;
    // double acap,bcap;
    // double sin_th0_q1;
    // double m0,m1,m2,m3,r0,r1,r2,r3,l0,l1,l2,l3,mt;
    // double r0x,r0y;
    // double a,b,c,d;

    char cmd;
    bool isDone = false;
    bool legitChar = false;

    // m0 = 13.3; //ta evala se sxolio gia na valo tous kosta
    // m1 = 0.083;
    // m2 = 0.187;
    // m3 = 0.03547;
    // r0 = 0.1954;
    // r1 = 0.062;
    // r2 = 0.062;
    // r3 = 0.06553; //apo prakseis monos mou
    // l0 = 0;
    // l1 = 0.119;
    // l2 = 0.119;
    // l3 = 0.01947;
    // mt = 10;

    // r0x = 0.17271; //syntetagmenes tou shoulder joint se sxesh me vash, apo xacro ta phra
    // r0y = 0.091404;

    // a=sqrt(r0x*r0x+r0y*r0y);
    // b=l1+r1;
    // c=l2+r2;
    // d=l3+r3;

    // xE_in=0.72;
    // yE_in=-0.25143;
    // thetaE_in=0.7854;
    // q01=-30*M_PI/180;

    // xb_in=0;
    // yb_in=0;
    // thetab_in=0*(M_PI/180)+q01;

    // cos_q2=((pow((xE_in-xb_in-a*cos(thetab_in)-d*cos(thetaE_in)),2))+
    //     (pow((yE_in-yb_in-a*sin(thetab_in)-d*sin(thetaE_in)),2))-(b*b)-(c*c))/(2*b*c);
    // sin_q2=sqrt(1-pow((cos_q2),2));

    // acap=c*sin_q2;
    // bcap=b+c*cos_q2;

    // sin_th0_q1=-((acap*(xE_in-xb_in-a*cos(thetab_in)-d*cos(thetaE_in))-
    //         bcap*(yE_in-yb_in-a*sin(thetab_in)-d*sin(thetaE_in)))/((b*b)+(c*c)+2*b*c*cos_q2));
    // cos_th0_q1=(acap*(yE_in-yb_in-a*sin(thetab_in)-d*sin(thetaE_in))+
    //     bcap*(xE_in-xb_in-a*cos(thetab_in)-d*cos(thetaE_in)))/((b*b)+(c*c)+2*b*c*cos_q2);
    
    // q2_in=atan2(sin_q2,cos_q2);
    // q1_in=atan2(sin_th0_q1,cos_th0_q1)-thetab_in;
    // q3_in=thetaE_in-thetab_in-q1_in-q2_in;



    ROS_INFO("[testsetter]: Warning! Do not proceed before running the Gazebo. \n");
    while(!isDone){

        isDone = false;
        legitChar = false;

        std::cout <<"Give me initial q1(shoulder) (deg):" <<std::endl;
        std::cin >> q1;
        std::cout <<"Give me initial q2(elbow) (deg):" <<std::endl;
        std::cin >> q2;
        std::cout <<"Give me initial q3(wrist) (deg):" <<std::endl;
        std::cin >> q3;

        q1 = q1*M_PI/180; //from deg to rad
        q2 = q2*M_PI/180; //from deg to rad
        q3 = q3*M_PI/180; //from deg to rad

        msg_LS.data = q1;
        msg_LE.data = q2;
        msg_LW.data = q3;

        LS_position_pub.publish(msg_LS);
        LE_position_pub.publish(msg_LE);
        LW_position_pub.publish(msg_LW);

        ros::spinOnce();  
        while(!legitChar){
            std::cout<<"Press Y to continue or N to give new values. "<<std::endl;
            std::cin>>cmd;
            if(cmd == 'Y'){
                legitChar = true;
                isDone = true;
            }
            if(cmd == 'N'){
                legitChar = true;
                isDone = false;
            }
        }
    }

    ROS_INFO("[testsetter]: Initial pose of Cepheus established. \n");
    
    ros::Duration(2.0).sleep();
    //////////////////////////CONTROLLER SWITCHING////////////////////////////////

    // Step 1: Stop controllers A1, A2, A3
    if (!switchControllers(nh, controllersA, {})) {
        ROS_ERROR("Failed to stop controllers A1, A2, A3");
        return 1;
    }

    // Step 2: Unload controllers A1, A2, A3
    for (const auto& controller : controllersA) {
        if (!unloadController(nh, controller)) {
            ROS_ERROR("Failed to unload controller %s", controller.c_str());
            return 1;
        }
    }

    // Step 3: Load controllers B1, B2, B3
    for (const auto& controller : controllersB) {
        if (!loadController(nh, controller)) {
            ROS_ERROR("Failed to load controller %s", controller.c_str());
            return 1;
        }
    }

    // Step 4: Start controllers B1, B2, B3
    if (!switchControllers(nh, {}, controllersB)) {
        ROS_ERROR("Failed to start controllers B1, B2, B3");
        return 1;
    }

    ROS_INFO("Successfully switched from position controllers to effort controllers.");

    ros::spinOnce();
    return 0;
}
