#ifndef MY_HEADER2_H
#define MY_HEADER2_H
#include "includes.h"


/////////////// GLOBAL VARIABLES INITIALIZATION START////////////////////////
/*Boolean flags*/
bool reachedTarget = false;
bool start_movement = false;
bool firstTime = true; //boolean for first time listening

/*Cepheus' variables*/
double q1;       // angle of first joint [rad] from callback
double q2;       // angle of second joint [rad] from callback
double q3, q3dot;
double q1dot;    // rate of first joint [rad/s] from callback
double q2dot;    // rate of second joint [rad/s] from callback
double theta0dot;   // reaction wheel velocity [rad/s]  h allios theta0dot
double theta0;
double force_x,force_y,torque_z;
double ee_x, ee_y; //ee_Z not needed
double xc0, yc0, xc0dot, yc0dot; //center of mass of base
double thetach; //orientation of chaser (end effector)

/*Target's variables*/
double ring_x, ring_y;
double xt,yt;  //xtarget, ytarget, pros to paron einai idia me to ring_x,ring_y
double thetat; //angle of target (orientation)



/*Arxikes (kai telikes) sinthikes*/
double q1_init = -60 * (M_PI / 180);
double q2_init = 105 * (M_PI / 180);
double q3_init = 45 * (M_PI / 180);
double q1des = 45* (M_PI / 180);  //kanonika afta briskontai apo inverse kinematics alla tespa
double q2des = 45 * (M_PI / 180);
double q3des = 0;
double q1dotdes = 0.0;
double q2dotdes = 0.0;
///////////really needed//////////////
double xch_in, ych_in, thetach_in; //arxikh thesh chaser (end effector)
double xt_in, yt_in, thetat_in;     //arxikh thesh target (ring)

/*PD parameters*/
double Kp = 5;
double Kd = 0.5;



/* Initialize variables needed for the control loop*/
double errorq[3];
double error_qdot[3];
double torq[3];
double prev_torq[3];

/*Messages to publish */
std_msgs::Float64 msg_RW;
std_msgs::Float64 msg_LE;
std_msgs::Float64 msg_LS;


/*Impedance control variables*/
double sitffness = 100.0;
double damping = 5.0;
double imp_error[2];


/*Kinematics variables From alex thesis*/
double r0,m0,l0; //l0 diko moy, h aktina tou khfea
double r1,m1,l1;
double r2,m2,l2;
double r3,m3,l3;
double M;
double theta0, thetaee;
double a,b,c,d;
double ibzz;
double i1zz, i2zz, i3zz, itzz;


/*Needed for calculations*/
double a0x,a1x,a2x,a3x,a4x,a5x; //for s_x
double a0y,a1y,a2y,a3y,a4y,a5y; //for s_y
double a0t,a1t,a2t,a3t,a4t,a5t; //for s_theta
double s_x,s_y,s_theta;  //to polyonymo..prepei na to ftiakso
Eigen::VectorXd xch_in(xti,yti,0);
Eigen::VectorXd xch_c(0,0,0);
Eigen::VectorXd xdf(0,0,0);
Eigen::VectorXd xdc(xt,yt);
Eigen::VectorXd fext(0,0,0); //fx,fy,n
Eigen::VectorXd xd; //the ACTUAL DESIRED TRAJECTORY
Eigen::VectorXd c(6);
Eigen::VectorXd fact;
Eigen::VectorXd z;
Eigen::VectorXd zdot;
Eigen::MatrixXd w;
Eigen::MatrixXd jacobian(6,9);
Eigen::MatrixXd jacobiandot(6,9);
Eigen::MatrixXd h(6,6); //imp.thesis sel 62 eksisosi 5-31
Eigen::MatrixXd::Identity(6,6) i6;
Eigen::MatrixXd::Identity(3,3) i3;
Eigen::MatrixXd::Zero(6,6) z6;
Eigen::MatrixXd::Zero(3,3) z3;
Eigen::MatrixXd e;



/*Jacobian coefficients*/
double j13, j14, j15, j16, j23, j24, j25, j26;

/*Jacobiandot coefficients*/
double j13dot, j14dot, j15dot, j16dot, j23dot, j24dot, j25dot, j26dot;

/*c coefficients*/
double c11, c21, c31, c41, c51, c61;

/*h coefficients*/
double h11, h12, h13, h14, h15, h16,
        h21, h22, h23, h24, h25, h26,
        h31, h32, h33, h34, h35, h36,
        h41, h42, h43, h44, h45, h46,
        h51, h52, h53, h54, h55, h56,
        h61, h62, h63, h64, h65, h66;





/*Misc, maybe useless*/ 
int currentMeasurement = 0;
double frequency = (float)1/DT;
//double qd[3];
//double qd_dot[3];





/////////////// GLOBAL VARIABLES INITIALIZATION END////////////////////////

#endif