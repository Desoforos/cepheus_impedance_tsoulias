#ifndef MY_HEADER2_H
#define MY_HEADER2_H
#include "includes.h"


/////////////// GLOBAL VARIABLES DECLARATION START////////////////////////
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
// std_msgs::Float64 msg_TX; //thruster_x
// std_msgs::Float64 msg_TY; //thruster_y
geometry_msgs::Wrench base_wrench; //x,y force of thrusters
std_msgs::Float64 msg_xd_x; //xd_x
std_msgs::Float64 msg_xd_y; //xd_y



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
double mt;
double r0x,r0y;
//double a,b,c,d; not needed
double ibzz;
double i1zz, i2zz, i3zz, itzz;


/*Needed for calculations*/
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
Eigen::VectorXd fext(3); // << (0,0,0); //fx,fy,n
Eigen::VectorXd xd(3); //the  DESIRED TRAJECTORY xEd,yEd,thetaEd
Eigen::VectorXd xddot(3); //xEddot, yEddot, thetaEddot
Eigen::VectorXd xddotdot(3); //xEddotdot, yEddotdot, thetaEddotdot
Eigen::VectorXd xfd(3); //the  DESIRED TRAJECTORY xEd,yEd,thetaEd
Eigen::VectorXd xfddot(3); //xEddot, yEddot, thetaEddot
Eigen::VectorXd xfddotdot(3); //xEddotdot, yEddotdot, thetaEddotdot
Eigen::VectorXd xcd(3); //the  DESIRED TRAJECTORY xEd,yEd,thetaEd
Eigen::VectorXd xcddot(3); //xEddot, yEddot, thetaEddot
Eigen::VectorXd xcddotdot(3); //xEddotdot, yEddotdot, thetaEddotdot
Eigen::VectorXd xee = Eigen::VectorXd::Zero(3); //(0,0,0) //the actual trajecotry (x,y,theta)
Eigen::VectorXd xeedot = Eigen::VectorXd::Zero(3);  //(0,0,0)//the actual trajecotry (x,y,theta)
Eigen::VectorXd c(6);
Eigen::VectorXd fact(3);
Eigen::VectorXd z(6);
Eigen::VectorXd zdot(6);
Eigen::MatrixXd w(3,3);
Eigen::MatrixXd jacobian(3,6);
Eigen::MatrixXd jacobiandot(3,6);
Eigen::MatrixXd h(6,6); //imp.thesis sel 62 eksisosi 5-31
Eigen::MatrixXd i6 = Eigen::MatrixXd::Identity(6,6);
Eigen::MatrixXd i3 = Eigen::MatrixXd::Identity(3,3);
Eigen::MatrixXd z6 = Eigen::MatrixXd::Zero(6,6);
Eigen::MatrixXd z3 = Eigen::MatrixXd::Zero(3,3);
Eigen::VectorXd e(3); //error for x,y,theta
Eigen::VectorXd edot(3);
Eigen::VectorXd edotdot(3);
Eigen::MatrixXd kd = 100*Eigen::MatrixXd::Identity(6,6); //esvhsa to (6,6)
Eigen::VectorXd a_x(6);
Eigen::VectorXd a_y(6);
Eigen::VectorXd a_theta(6);
Eigen::VectorXd b1_x(6);
Eigen::VectorXd b1_y(6);
Eigen::VectorXd b1_theta(6);
Eigen::MatrixXd a_matrix = Eigen::MatrixXd::Identity(6,6);
Eigen::VectorXd fdes(3); //(0.1,0,0);
Eigen::VectorXd fdes_star(3);
Eigen::MatrixXd ke_star=10000*Eigen::MatrixXd::Identity(3,3);
Eigen::MatrixXd kd_e(3,3);
Eigen::MatrixXd kd_b(3,3);
Eigen::MatrixXd md(6,6);
Eigen::MatrixXd md_e(3,3);
Eigen::MatrixXd md_b(3,3);
Eigen::MatrixXd bd(6,6);
Eigen::MatrixXd bd_e(3,3);
Eigen::MatrixXd bd_b(3,3);
Eigen::MatrixXd je(3,6); //second jacobian for the qact = je*Fact
Eigen::VectorXd qact(6); //fx,fy,ns,t1,t2,t3
Eigen::VectorXd rEddotdot(3);








double a1 = 1000; //for xd anti gia 10000
double a2 = 0.001; //anti gia 0.0001
double t0 =0 , t_free =50; //anti gia 200
double z_free = 1;
double ts_free = 0.1*t_free;
double wn_free = 6/ts_free;
double z_contact;
double wn_contact;
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

/*JE coefficients*/
double je11, je12, je13, je14, je15, je16;
double je21, je22, je23, je24, je25, je26;
double je31, je32, je33, je34, je35, je36;




/*Misc, maybe useless*/ 
int currentMeasurement = 0;
double frequency = (float)1/DT;
//double qd[3];
//double qd_dot[3];





/////////////// GLOBAL VARIABLES DECLARATION END////////////////////////

#endif