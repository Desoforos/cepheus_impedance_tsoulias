#ifndef ROBOT_VARIABLES_H
#define ROBOT_VARIABLES_H
#include "includes.h"


/////////////// GLOBAL VARIABLES DECLARATION START////////////////////////
/*Boolean flags*/
bool reachedTarget;// = false;
bool start_movement;// = false;
// bool eefirstTime;// = true; //boolean for first time listening
// bool targetfirstTime;
// bool basefirstTime;
bool firstTime;

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
double ee_x, ee_y; //ee_Z not needed
double xc0, yc0, xc0dot, yc0dot; //center of mass of base
double thetach; //orientation of chaser (end effector)
double xE_prev,yE_prev, thetaE_prev;
double xt_prev, yt_prev, thetat_prev;
double xc0_prev, yc0_prev, theta0_prev;


/*Target's variables*/
double xt,yt;  //xtarget, ytarget, pros to paron einai idia me to ring_x,ring_y
double thetat; //angle of target (orientation)
double rawxt, rawyt, rawthetat;
double xtdot, ytdot, thetatdot;
double rawxtdot, rawytdot, rawthetatdot;
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
std_msgs::Float64 msg_fextx_raw;
// std_msgs::Float64 msg_TX; //thruster_x
// std_msgs::Float64 msg_TY; //thruster_y
geometry_msgs::Wrench base_wrench; //x,y force of thrusters
std_msgs::Float64 msg_xd_x; //xd_x
std_msgs::Float64 msg_xd_y; //xd_y
std_msgs::Float64 msg_xd_theta; //xd_y

std_msgs::Float64 msg_xt_x; 
std_msgs::Float64 msg_xt_y; 
std_msgs::Float64 msg_xt_theta; 

std_msgs::Float64 msg_xt_x_raw; 
std_msgs::Float64 msg_xt_y_raw; 
std_msgs::Float64 msg_xt_theta_raw; 

std_msgs::Float64 msg_xee_x; 
std_msgs::Float64 msg_xee_y; 
std_msgs::Float64 msg_xee_theta; 

std_msgs::String arduino_msg; 
std_msgs::Bool start_moving;
std_msgs::Bool start_grab_msg;






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

// int contactCounter = 0;

bool firstTimeq1 = true;
bool firstTimeq2 = true;
bool firstTimeq3 = true;
bool offsetsdone = false;
double offsetq1 = 0.0;
double offsetq2 = 0.0;
double offsetq3 = 0.0;
double q1known = 1.84447;  //gonies analoga me tin gnosti diataksi, ego ekana omos thetiko akro, agonas karpos arnitika akra
double q2known = -0.9542;
double q3known = -0.2856;

std_msgs::Float64 msg_q1;
std_msgs::Float64 msg_q2;
std_msgs::Float64 msg_q3;
std_msgs::Float64 msg_theta0;

std_msgs::Float64 msg_q1dot;
std_msgs::Float64 msg_q2dot;
std_msgs::Float64 msg_q3dot;
std_msgs::Float64 msg_theta0dot;


std_msgs::Float64 msg_torqueq1;
std_msgs::Float64 msg_torqueq2;
std_msgs::Float64 msg_torqueq3;
std_msgs::Float64 msg_torquerw;

Eigen::VectorXd tau(4);
Eigen::VectorXd prev_tau(4);


std_msgs::Float64 msg_xd_x_dot; //xd_x
std_msgs::Float64 msg_xd_y_dot; //xd_y
std_msgs::Float64 msg_xd_theta_dot; //xd_y
std_msgs::Float64 msg_xd_theta0_dot; //xd_y


std_msgs::Float64 msg_xt_x_dot; 
std_msgs::Float64 msg_xt_y_dot; 
std_msgs::Float64 msg_xt_theta_dot; 

std_msgs::Float64 msg_xt_x_dot_raw; 
std_msgs::Float64 msg_xt_y_dot_raw; 
std_msgs::Float64 msg_xt_theta_dot_raw; 

std_msgs::Float64 msg_xt_theta0_dot; 

std_msgs::Float64 msg_xee_x_dot; 
std_msgs::Float64 msg_xee_y_dot; 
std_msgs::Float64 msg_xee_theta_dot; 
std_msgs::Float64 msg_xee_theta0_dot; 

std_msgs::Float64 msg_xt_theta0, msg_xd_theta0, msg_xee_theta0;

bool safeclose = false;
double maxtorque = 10;
double theta0safeclose;// = 0;  //telikes synthikes gia safeclose
double q1safeclose ;//= 45*M_PI/180; 
double q2safeclose ;//= 45*M_PI/180; 
double q3safeclose ;//= 10*M_PI/180;
double xsafeclose, ysafeclose,thetasafeclose;

double sumq1 = 0, sumq2 = 0, sumq3 =0;
double sumq1dot = 0, sumq2dot = 0, sumq3dot =  0;
double force_x, raw_force_x;
double forcesum = 0;
int force_window_size = 10;
int q_window_size = 10;
double sumxdot = 0, sumydot = 0, sumthetadot =  0;
double sumtheta0dot = 0;
double sumxtdot =0, sumytdot =0, sumthetatdot = 0;


std::deque<double> q1_window;  // Stores the last N values
std::deque<double> q2_window;  // Stores the last N values
std::deque<double> q3_window;  // Stores the last N values

std::deque<double> q1dot_window;  // Stores the last N values
std::deque<double> q2dot_window;  // Stores the last N values
std::deque<double> q3dot_window;  // Stores the last N values

std::deque<double> xdot_window;  // Stores the last N values
std::deque<double> ydot_window;  // Stores the last N values
std::deque<double> thetadot_window;  // Stores the last N values

std::deque<double> xtdot_window;  // Stores the last N values
std::deque<double> ytdot_window;  // Stores the last N values
std::deque<double> thetatdot_window;  // Stores the last N values

std::deque<double> xc0dot_window;  // Stores the last N values
std::deque<double> yc0dot_window;  // Stores the last N values
std::deque<double> theta0dot_window;


std::deque<double> force_window; 



Eigen::MatrixXd kp_multiplier(4,4);
Eigen::MatrixXd bd_multiplier(4,4);

bool targetcheck, eecheck, basecheck;

bool show = false;

double xdotprev = 0, ydotprev = 0, thetadotprev = 0;
double xtdotprev = 0, ytdotprev = 0, thetatdotprev = 0;
double xc0dotprev = 0, yc0dotprev = 0, theta0dotprev = 0;

std::deque<double> xwindow;  // Stores the last N values
std::deque<double> ywindow;  // Stores the last N values
std::deque<double> thetawindow;  // Stores the last N values
std::deque<double> theta0window;  // Stores the last N values

double sumx = 0, sumy = 0, sumtheta = 0, sumtheta0 = 0;

std::deque<double> xhistory;  // Stores the last N values
std::deque<double> yhistory;  // Stores the last N values
std::deque<double> thetahistory;  
 
 double sumxc0dot = 0, sumyc0dot = 0 ;
 double filtered_ydot = 0;
 bool stopMotors = false;
 bool grabStarted = false;
/////////////// GLOBAL VARIABLES DECLARATION END////////////////////////

#endif