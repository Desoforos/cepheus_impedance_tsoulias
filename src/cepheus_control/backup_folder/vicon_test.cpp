#include "includes.h"
#include <typeinfo>

double ee_x, ee_y, thetach,xE_in,yE_in, thetaE_in;
double xt, yt, thetat, xt_in, yt_in, thetat_in;
double eefirstTime = true;
double targetfirstTime = true;
double xE_prev , yE_prev ,thetaE_prev;
double xt_prev, yt_prev, thetat_prev;
double xE_vel, yE_vel, thetaE_vel;
double xt_vel, yt_vel, thetat_vel;


void ee_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    if(!eefirstTime){
        xE_prev = ee_x;
        yE_prev = ee_y;
        thetaE_prev = thetach;
    }
	ee_x = msg->transform.translation.x;
	ee_y = msg->transform.translation.y;
	tf::Quaternion qee( //for angle of ee
		msg->transform.rotation.x,
		msg->transform.rotation.y,
		msg->transform.rotation.z,
		msg->transform.rotation.w);
    tf::Matrix3x3 m_ee(qee);	
    double rollee, pitchee, yawee;
	m_ee.getRPY(rollee, pitchee, yawee);
	thetach = yawee; //angle of chaser(ee)
	if(eefirstTime){   //initialize the postiion of chaser and target for the first time ONLY
		xE_in = ee_x;
		yE_in = ee_y;
		thetaE_in = thetach; 
		eefirstTime = false;
        std::cout<<"Initial position of end effector is: xe_in: "<<xE_in<<" yE_in: "<<yE_in<<" thetaE_in: "<<thetaE_in<<std::endl;
		}
}


void target_posCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    if(!targetfirstTime){
        xt_prev = xt;
        yt_prev = yt;
        thetat_prev = thetat;
    }
	xt = msg->transform.translation.x;
	yt = msg->transform.translation.y;
	tf::Quaternion qt( //for angle of ee
		msg->transform.rotation.x,
		msg->transform.rotation.y,
		msg->transform.rotation.z,
		msg->transform.rotation.w);
    tf::Matrix3x3 m_t(qt);	
    double rollt, pitcht, yawt;
	m_t.getRPY(rollt, pitcht, yawt);
	thetat = yawt; //angle of chaser(ee)
	if(targetfirstTime){   //initialize the postiion of chaser and target for the first time ONLY
		xt_in = xt;
		yt_in = yt;
		thetat_in = thetat;
		targetfirstTime = false;
        std::cout<<"Initial position of target is: xt_in: "<<xt_in<<" yt_in: "<<yt_in<<" thetat_in: "<<thetat_in<<std::endl;
		}
}

void updateVel(double dt){
    xE_vel = (ee_x-xE_prev)/dt;
    yE_vel = (ee_y-yE_prev)/dt;
    thetaE_vel = (thetach-thetaE_prev)/dt;

    xt_vel = (xt-xt_prev)/dt;
    yt_vel = (yt-yt_prev)/dt;
    thetat_vel = (thetat-thetat_prev)/dt;

}

int main(int argc, char **argv) {


    /* ros init */
    ros::init(argc, argv, "vicon_test_node");
    ros::NodeHandle nh;


    ros::Subscriber ee_pos_sub = nh.subscribe("/vicon/end_effector_new/end_effector_new", 1, ee_posCallback);
    ros::Subscriber target_pos_sub = nh.subscribe("/vicon/target_new/target_new", 1, target_posCallback);



    ros::Rate loop_rate(100);
    int counter = 0;
    

    while(ros::ok()){
        ros::spinOnce();
        updateVel(0.01);
        if(counter%100 == 0){
            std::cout<<"end effector position is: ee_x: "<<ee_x<<" ee_y: "<<ee_y<<" thetach: "<<thetach<<std::endl;
            std::cout<<"target position is: xt: "<<xt<<" yt: "<<yt<<" thetat: "<<thetat<<std::endl;
            std::cout<<" "<<std::endl;

            std::cout<<"end effector velocity is: xE_vel: "<<xE_vel<<" yE_vel: "<<yE_vel<<" thetaE_vel: "<<thetaE_vel<<std::endl;
            std::cout<<"target velocity is: xt_vel: "<<xt_vel<<" yt_vel: "<<yt_vel<<" thetat_vel: "<<thetat_vel<<std::endl;
            std::cout<<"///////////////////////////////"<<std::endl;
        }
        counter++;
        loop_rate.sleep();
    }

    return 0;
}