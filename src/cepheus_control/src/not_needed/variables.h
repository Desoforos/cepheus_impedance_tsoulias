#ifndef MY_HEADER2_H
#define MY_HEADER2_H
#include "includes.h"


/////////////// GLOBAL VARIABLES DECLARATION START////////////////////////
/*Boolean flags*/
bool reachedTarget;// = false;
bool start_movement;// = false;
bool firstTime;// = true; //boolean for first time listening x,y
bool qfirstTime;
bool wristInitialised;

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

/*Target's variables*/
double ring_x, ring_y;
double xt,yt;  //xtarget, ytarget, pros to paron einai idia me to ring_x,ring_y
double thetat; //angle of target (orientation)
double xtdot, ytdot, thetatdot;



/*Arxikes (kai telikes) sinthikes*/
double q1_init;// = -60 * (M_PI / 180);
double q2_init; // = 105 * (M_PI / 180);
double q3_init; // = 45 * (M_PI / 180);
double q1des; // = 45* (M_PI / 180);  //kanonika afta briskontai apo inverse kinematics alla tespa
double q2des; // = 45 * (M_PI / 180);
double q3des; // = 0;
double q1dotdes; // = 0.0;
double q2dotdes; // = 0.0;
///////////really needed//////////////
double xch_in, ych_in, thetach_in; //arxikh thesh chaser (end effector)
double xt_in, yt_in, thetat_in;     //arxikh thesh target (ring)

/*PD parameters*/
// double Kp = 5;
// double Kd = 0.5;



/* Initialize variables needed for the control loop*/
double errorq[3];
double error_qdot[3];
double torq[3]; //might be not needed
double prev_torq[3];

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
std_msgs::Float64 msg_xd_theta0; //xd_y


std_msgs::Float64 msg_xt_x; 
std_msgs::Float64 msg_xt_y; 
std_msgs::Float64 msg_xt_theta; 
std_msgs::Float64 msg_xt_theta0; //xd_y


std_msgs::Float64 msg_xee_x; 
std_msgs::Float64 msg_xee_y; 
std_msgs::Float64 msg_xee_theta; 
std_msgs::Float64 msg_xee_theta0; //xd_y


std_msgs::Float64 msg_xd_x_dot; //xd_x
std_msgs::Float64 msg_xd_y_dot; //xd_y
std_msgs::Float64 msg_xd_theta_dot; //xd_y
std_msgs::Float64 msg_xd_theta0_dot; //xd_y


std_msgs::Float64 msg_xt_x_dot; 
std_msgs::Float64 msg_xt_y_dot; 
std_msgs::Float64 msg_xt_theta_dot; 
std_msgs::Float64 msg_xt_theta0_dot; 

std_msgs::Float64 msg_xee_x_dot; 
std_msgs::Float64 msg_xee_y_dot; 
std_msgs::Float64 msg_xee_theta_dot; 
std_msgs::Float64 msg_xee_theta0_dot; 


std_msgs::Float64 msg_torqueq1;
std_msgs::Float64 msg_torqueq2;
std_msgs::Float64 msg_torqueq3;
std_msgs::Float64 msg_torquerw;







/*Impedance control variables*/
double sitffness = 100.0;
double damping = 5.0;
double imp_error[2];


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


/*Needed for calculations*/
// double a, b, c, d; den nomizo na ta xreiazomaste
double a0x,a1x,a2x,a3x,a4x,a5x; //for s_x
double a0y,a1y,a2y,a3y,a4y,a5y; //for s_y
double a0t,a1t,a2t,a3t,a4t,a5t; //for s_theta
double s_x,s_y,s_theta;  //to polyonymo
double sdot_x, sdot_y, sdot_theta;
double sdotdot_x, sdotdot_y, sdotdot_theta;
double xfEd, yfEd, thetafEd;
double xfEddot, yfEddot, thetafEddot;
double xfEddotdot, yfEddotdot, thetafEddotdot;
double xcEd, ycEd, thetacEd;
double xcEddot, ycEddot, thetacEddot;
double xcEddotdot, ycEddotdot, thetacEddotdot;
//Eigen::VectorXd xch_in(xti,yti,0); mallon den xreiazetai
Eigen::VectorXd xch_c(3);//xch_c <<0 , 0 , 0;
// Eigen::VectorXd xdf(3); //xdf<<0 , 0 ,0 ;
// Eigen::VectorXd xdc(3); // <<xt,yt,thetat;
Eigen::Vector3d fext(0,0,0);// = Eigen::VectorXd::Zero(3); // << (0,0,0); //fx,fy,n
Eigen::Vector3d xd(0,0,0);// = Eigen::VectorXd::Zero(3); //the  DESIRED TRAJECTORY xEd,yEd,thetaEd
Eigen::Vector3d xddot(0,0,0);// = Eigen::VectorXd::Zero(3); //xEddot, yEddot, thetaEddot
Eigen::Vector3d xddotdot(0,0,0);// = Eigen::VectorXd::Zero(3); //xEddotdot, yEddotdot, thetaEddotdot
Eigen::Vector3d xfd(0,0,0);// = Eigen::VectorXd::Zero(3); //the  DESIRED TRAJECTORY xEd,yEd,thetaEd
Eigen::Vector3d xfddot(0,0,0);// = Eigen::VectorXd::Zero(3); //xEddot, yEddot, thetaEddot
Eigen::Vector3d xfddotdot(0,0,0);// = Eigen::VectorXd::Zero(3); //xEddotdot, yEddotdot, thetaEddotdot
Eigen::Vector3d xcd(0,0,0);// = Eigen::VectorXd::Zero(3);//the  DESIRED TRAJECTORY xEd,yEd,thetaEd
Eigen::Vector3d xcddot(0,0,0);// = Eigen::VectorXd::Zero(3); //xEddot, yEddot, thetaEddot
Eigen::Vector3d xcddotdot(0,0,0);// = Eigen::VectorXd::Zero(3); //xEddotdot, yEddotdot, thetaEddotdot
Eigen::Vector3d xee(0,0,0);// = Eigen::VectorXd::Zero(3); //(0,0,0) //the actual trajecotry (x,y,theta)
Eigen::Vector3d xeedot(0,0,0);// = Eigen::VectorXd::Zero(3);  //(0,0,0)//the actual trajecotry (x,y,theta)
Eigen::VectorXd c(6);// = Eigen::VectorXd::Zero(6) ;
Eigen::Vector3d fact(0,0,0);// = Eigen::VectorXd::Zero(3);;
Eigen::VectorXd z(6);//= Eigen::VectorXd::Zero(6); //real
Eigen::VectorXd zdot(6);// = Eigen::VectorXd::Zero(6); //real
Eigen::VectorXd zddotdot(6);// = Eigen::VectorXd::Zero(6);//prosoxh einai desired!
Eigen::MatrixXd w = Eigen::MatrixXd::Zero(3,3);;
Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6,6); //anti gia 3x6 tora einai h jacobian gia 33 kai gia base
Eigen::MatrixXd jacobiandot = Eigen::MatrixXd::Zero(6,6);
Eigen::MatrixXd h = Eigen::MatrixXd::Zero(6,6); //imp.thesis sel 62 eksisosi 5-31
Eigen::MatrixXd i6 = Eigen::MatrixXd::Identity(6,6);
Eigen::MatrixXd i3 = Eigen::MatrixXd::Identity(3,3);
Eigen::MatrixXd z6 = Eigen::MatrixXd::Zero(6,6);
Eigen::MatrixXd z3 = Eigen::MatrixXd::Zero(3,3);
Eigen::VectorXd e(6);// = Eigen::VectorXd::Zero(6); //error for x,y,theta
Eigen::VectorXd edot(6);// = Eigen::VectorXd::Zero(6); 
Eigen::VectorXd edotdot(6);// = Eigen::VectorXd::Zero(6); 
Eigen::MatrixXd kd(6,6); // = Eigen::MatrixXd::Zero(6,6);// = 100*Eigen::MatrixXd::Identity(6,6); //esvhsa to (6,6)
Eigen::VectorXd a_x(6);// = Eigen::VectorXd::Zero(6);
Eigen::VectorXd a_y(6);// = Eigen::VectorXd::Zero(6);
Eigen::VectorXd a_theta(6);// = Eigen::VectorXd::Zero(6);
Eigen::VectorXd b1_x(6);// = Eigen::VectorXd::Zero(6); mono afta sto calc.h
Eigen::VectorXd b1_y(6);// = Eigen::VectorXd::Zero(6);
Eigen::VectorXd b1_theta(6);// = Eigen::VectorXd::Zero(6);
Eigen::MatrixXd a_matrix = Eigen::MatrixXd::Identity(6,6);
Eigen::VectorXd fdes(3) ; //(0.1,0,0) arxikopoiheitai sthn initialise
// Eigen::VectorXd fdes_star(3);
Eigen::MatrixXd ke_star=100*Eigen::MatrixXd::Identity(3,3);
Eigen::MatrixXd kd_e(3,3); // = Eigen::MatrixXd::Zero(3,3);
Eigen::MatrixXd kd_b(3,3); // = Eigen::MatrixXd::Zero(3,3);
Eigen::MatrixXd md(6,6);// = Eigen::MatrixXd::Zero(6,6);
Eigen::MatrixXd md_e = 30000*Eigen::MatrixXd::Identity(3,3); //anti gia 30000
Eigen::MatrixXd md_b = 30000*Eigen::MatrixXd::Identity(3,3); //anti gia 30000
Eigen::MatrixXd bd(6,6);// = Eigen::MatrixXd::Zero(6,6);
Eigen::MatrixXd bd_e = Eigen::MatrixXd::Zero(3,3);
Eigen::MatrixXd bd_b = Eigen::MatrixXd::Zero(3,3);
Eigen::MatrixXd je = Eigen::MatrixXd::Zero(3,6); //update: jacobian for ee
Eigen::MatrixXd jedot = Eigen::MatrixXd::Zero(3,6); //update: jacobian for ee
Eigen::VectorXd qact(6);// =  Eigen::VectorXd::Zero(6);  //fx,fy,ns,t1,t2,t3
Eigen::Vector3d rEddotdot(0,0,0);// =  Eigen::VectorXd::Zero(3); 








// double a1 = 1000; //for xd anti gia 10000
// double a2 = 0.001; //anti gia 0.0001
double t0 =0 , t_free; //anti gia 200
double z_free = 1;
double ts_free = 0.1*t_free;
double wn_free = 6/ts_free;
//double z_contact;
//double wn_contact;
double thetaE_in; //=30*(M_PI/180);

double v;
double x_target_in, y_target_in , theta_target_in;
double xE_in, yE_in;
///////initial sinthikes///////////////
double sin_x = 0, sdotin_x = 0, sdotdotin_x = 0;
double sin_y = 0, sdotin_y = 0, sdotdotin_y = 0;
double sin_theta = 0, sdotin_theta = 0, sdotdotin_theta = 0;
///////telikes sinthikes/////////////
double xE_contact; // = x_target_in - l0;
double yE_contact; // = y_target_in;
double thetaE_contact; // = theta_target_in;
double sfin_x = 1, sdotfin_x, sdotdotfin_x = 0;
double sfin_y = 1, sdotfin_y = 0, sdotdotfin_y = 0;
double sfin_theta = 1, sdotfin_theta = 0, sdotdotfin_theta = 0;










/*Misc, maybe useless*/ 
int currentMeasurement = 0;
double frequency = (float)1/DT;
//double qd[3];
//double qd_dot[3];

// /////////////EXTRA KOSTAS PARAMETERS////////////////////
// double w = 2; mallon no need
double z_cont =1 , ts_cont = 5, wn_cont = 6/ts_cont;
double q01 = -27.88931 * M_PI/180; //gonia tou shoulder joint se sxesh me base
double lt;
double s01 = 0.5, s02 = 0.2;
double i0zz;
/*Chaser's variables*/
double xbd,xbddot,xbddotdot; //mallon axrhsta
double ybd, ybddot, ybddotdot;
double thetabd, thetabddot, thetabddotdot;
Eigen::Vector3d xd_b(0,0,0);// = Eigen::VectorXd::Zero(3); //for base
Eigen::Vector3d xd_bdot(0,0,0);// = Eigen::VectorXd::Zero(3); 
Eigen::Vector3d xd_bdotdot(0,0,0);// = Eigen::VectorXd::Zero(3); 
Eigen::Vector3d fdes_ee(0,0,0);// = Eigen::VectorXd::Zero(3); 
Eigen::Vector3d fdes_b(0,0,0);// = Eigen::VectorXd::Zero(3);
Eigen::VectorXd fdes_star(6);// = Eigen::VectorXd::Zero(6); 
Eigen::VectorXd fext_star(6);// = Eigen::VectorXd::Zero(6); 
Eigen::VectorXd qext(6);// = Eigen::VectorXd::Zero(6); 
Eigen::MatrixXd jb(3,6);// = Eigen::MatrixXd::Zero(3,6); //jacobian of base
Eigen::MatrixXd jbdot = Eigen::MatrixXd::Zero(3,6);//= Eigen::MatrixXd::Zero(3,6); //jacobian of base

/*for diagnostics*/
Eigen::VectorXd vec1(6);
Eigen::VectorXd vec2(6);
Eigen::VectorXd vec3(6);
Eigen::MatrixXd mat(6,6);

double offset = 0.09/2;

/*for base controller*/
double fx = 0,fy = 0,ns =0 ;
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

Eigen::VectorXd tau(4);
Eigen::VectorXd prev_tau(4);

int steadycounter = 0;

double prevxtdot, prevytdot, prevthetatdot;

double torqueRW,torqueq1,torqueq2,torqueq3;
bool safeclose ;
double maxtorque = 20;
double theta0safeclose;// = 0;  //telikes synthikes gia safeclose
double q1safeclose ;//= 45*M_PI/180; 
double q2safeclose ;//= 45*M_PI/180; 
double q3safeclose ;//= 10*M_PI/180;



 
/////////////// GLOBAL VARIABLES DECLARATION END////////////////////////

#endif