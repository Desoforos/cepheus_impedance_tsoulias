#ifndef ROBOT_VARIABLES_H
#define ROBOT_VARIABLES_H
#include "includes.h"


/////////////// GLOBAL VARIABLES DECLARATION START////////////////////////
/*Boolean flags*/
bool reachedTarget;// = false;
bool start_movement;// = false;
bool eefirstTime;// = true; //boolean for first time listening
bool targetfirstTime;
bool basefirstTime;

bool ls_initialized = false;
bool le_initialized = false;
bool re_initialized = false;

double q1_init = -60 * (M_PI / 180);
double q2_init = 105 * (M_PI / 180);
double q3_init = 45 * (M_PI / 180);

/*Cepheus' variables*/
double q1;       // angle of first joint [rad] from callback
double q2;       // angle of second joint [rad] from callback
double q3, q3dot;
double q1dot;    // rate of first joint [rad/s] from callback
double q2dot;    // rate of second joint [rad/s] from callback
double theta0dot;   // reaction wheel velocity [rad/s]  h allios theta0dot
double theta0,thetaee;
double force_x,force_y,torque_z;
double ee_x, ee_y; //ee_Z not needed
double xc0, yc0, xc0dot, yc0dot; //center of mass of base
double thetach; //orientation of chaser (end effector)
double xE_prev,yE_prev, thetaE_prev;
double xt_prev, yt_prev, thetat_prev;
double xc0_prev, yc0_prev, theta0_prev;


/*Target's variables*/
double xt,yt;  //xtarget, ytarget, pros to paron einai idia me to ring_x,ring_y
double thetat; //angle of target (orientation)
double xtdot, ytdot, thetatdot;
double xt_in, yt_in, thetat_in;     //arxikh thesh target (ring)

/*Messages to publish */
std_msgs::Float64 msg_RW; //reaction wheel
std_msgs::Float64 msg_LS; //left shoulder
std_msgs::Float64 msg_LE; //left elbow
std_msgs::Float64 msg_LW; //left wrist
std_msgs::Float64 msg_ex; //errorx
std_msgs::Float64 msg_ey; //errory
std_msgs::Float64 msg_etheta; //errortheta
std_msgs::Float64 msg_fextx;
// std_msgs::Float64 msg_TX; //thruster_x
// std_msgs::Float64 msg_TY; //thruster_y
geometry_msgs::Wrench base_wrench; //x,y force of thrusters
std_msgs::Float64 msg_xd_x; //xd_x
std_msgs::Float64 msg_xd_y; //xd_y
std_msgs::Float64 msg_xd_theta; //xd_y

std_msgs::Float64 msg_xt_x; 
std_msgs::Float64 msg_xt_y; 
std_msgs::Float64 msg_xt_theta; 

std_msgs::Float64 msg_xee_x; 
std_msgs::Float64 msg_xee_y; 
std_msgs::Float64 msg_xee_theta; 

std_msgs::String arduino_msg; 
std_msgs::Bool start_moving;





/*Kinematics variables From alex thesis*/
double r0,m0,l0; //l0 diko moy, to mhkos tou elathriou
double r1,m1,l1;
double r2,m2,l2;
double r3,m3,l3;
double M;
double mt;
double r0x,r0y;
//double a,b,c,d; not needed
double ibzz;
double i1zz, i2zz, i3zz, itzz;


Eigen::Vector3d fext(0,0,0);// = Eigen::VectorXd::Zero(3); // << (0,0,0); //fx,fy,n
Eigen::Vector3d xee(0,0,0);// = Eigen::VectorXd::Zero(3); //(0,0,0) //the actual trajecotry (x,y,theta)
Eigen::Vector3d xeedot(0,0,0);// = Eigen::VectorXd::Zero(3);  //(0,0,0)//the actual trajecotry (x,y,theta)
Eigen::VectorXd c(6);// = Eigen::VectorXd::Zero(6) ;
Eigen::MatrixXd h = Eigen::MatrixXd::Zero(6,6); //imp.thesis sel 62 eksisosi 5-31
Eigen::MatrixXd je = Eigen::MatrixXd::Zero(3,6); //update: jacobian for ee
Eigen::MatrixXd jedot = Eigen::MatrixXd::Zero(3,6); //update: jacobian for ee



double thetaE_in; //=30*(M_PI/180);
double xE_in, yE_in;
double q01 = 0; //to eftiaksa oste na einai mhden!  // -27.88931 * M_PI/180; //gonia tou shoulder joint se sxesh me base
double s01 = 0.5, s02 = 0.2;

//gains
double kpx = 0.5,kdx = 0.6;
double kpy = 0.5,kdy = 0.6;
double kpth = 0.5,kdth = 0.6;
//double kprop = 0.5, kder = 10; AFTA DOULEVOUN GIA TO APLO PD XORIS TRAJECTORY gia tis palies mazes klp
double kprop = 0.5 ,kder = 20;
double kprop_mb,kder_mb; //model based

//for polynomial
double a0,a1,a2,a3,a4,a5;
double xstep,ystep,thstep,theta0step;
double xstepdot, ystepdot, thstepdot,theta0stepdot;
double xstepdotdot, ystepdotdot, thstepdotdot,theta0stepdotdot;
double sd = 0.25; //safety distance
double theta0in, theta0fin;
double xE_contact; // = x_target_in - l0;
double yE_contact; // = y_target_in;
double thetaE_contact;

bool beginGrab = false;
bool incontact =false;
bool beginSoft = false;
bool softFinished = false;
bool beginHard = false;
bool hardFinished = false;

bool gripperListenedSoft = false;
bool gripperListenedHard = false;

int contactCounter = 0;


 
/////////////// GLOBAL VARIABLES DECLARATION END////////////////////////

#endif