#include "includes.h"
#include "variables.h"



void finaltrajectories(double t){
    double s,sdot, sdotdot;

    s = a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
    sdot = a1 + 2*a2*t + 3*a3*pow(t,2) + 4*a4*pow(t,3) + 5*a5*pow(t,4);
    sdotdot = 2*a2 + 6*a3*t + 12*a4*pow(t,2) + 20*a5*pow(t,3);
    double xstepfr, ystepfr, thstepfr, theta0stepfr;
    double xstepdotfr, ystepdotfr, thstepdotfr, theta0stepdotfr;
    double xstepdotdotfr, ystepdotdotfr, thstepdotdotfr, theta0stepdotdotfr;
    double xstepc, ystepc, thstepc, theta0stepc;
    double xstepdotc, ystepdotc, thstepdotc, theta0stepdotc;
    double xstepdotdotc, ystepdotdotc, thstepdotdotc, theta0stepdotdotc;
    double a11 = pow(10,7);
    double a22 = pow(10,-7);


    xstepfr = xE_in + s*(xt_in - xE_in);
    ystepfr = yE_in + s*(yt_in - yE_in);
    thstepfr = thetaE_in + s*(thetat_in - thetaE_in);
    theta0stepfr = theta0in + s*(theta0fin - theta0in);

    xstepdotfr = sdot*(xt_in-xE_in);
    ystepdotfr = sdot*(yt_in - yE_in);
    thstepdotfr = sdot*(thetat_in - thetaE_in);
    theta0stepdotfr =  sdot*(theta0fin - theta0in);

    xstepdotdotfr = sdotdot*(xt_in-xE_in);
    ystepdotdotfr = sdotdot*(yt_in - yE_in);
    thstepdotdotfr = sdotdot*(thetat_in - thetaE_in);
    theta0stepdotdotfr =  sdotdot*(theta0fin - theta0in);

    xstepc = xt_in;
    ystepc = yt_in;
    thstepc = thetat_in;
    theta0stepc = theta0fin;

    xstepdotc = 0;
    ystepdotc = 0;
    thstepdotc = 0;
    theta0stepdotc = 0;

    xstepdotdotc = 0;
    ystepdotdotc = 0;
    thstepdotdotc = 0;
    theta0stepdotdotc = 0;

    // xstep = xstepfr*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+xstepc*(abs(fext(0))/(abs(fext(0))+a22));
    // xstepdot = xstepdotfr*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+xstepdotc*(abs(fext(0))/(abs(fext(0))+a22));
    // xstepdotdot = xstepdotdotfr*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+xstepdotdotc*(abs(fext(0))/(abs(fext(0))+a22));

    // ystep = ystepfr*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+ystepc*(abs(fext(0))/(abs(fext(0))+a22));
    // ystepdot = ystepdotfr*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+ystepdotc*(abs(fext(0))/(abs(fext(0))+a22));
    // ystepdotdot = ystepdotdotfr*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+ystepdotdotc*(abs(fext(0))/(abs(fext(0))+a22));

    // thstep = thstepfr*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+thstepc*(abs(fext(0))/(abs(fext(0))+a22));
    // thstepdot = thstepdotfr*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+thstepdotc*(abs(fext(0))/(abs(fext(0))+a22));
    // thstepdotdot = thstepdotdotfr*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+thstepdotdotc*(abs(fext(0))/(abs(fext(0))+a22));

    // theta0step = theta0stepfr*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+theta0stepc*(abs(fext(0))/(abs(fext(0))+a22));
    // theta0stepdot = theta0stepdotfr*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+theta0stepdotc*(abs(fext(0))/(abs(fext(0))+a22));
    // theta0stepdotdot = theta0stepdotdotfr*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+theta0stepdotdotc*(abs(fext(0))/(abs(fext(0))+a22));

    if(abs(fext(0)<0.5)){
      xstep = xstepfr;
      ystep = ystepfr;
      thstep = thstepfr;
      theta0step = theta0stepfr;

      xstepdot = xstepdotfr;
      ystepdot = ystepdotfr;
      thstepdot = thstepdotfr;
      theta0stepdot = theta0stepdotfr;

      xstepdotdot = xstepdotdotfr;
      ystepdotdot = ystepdotdotfr;
      thstepdotdot = thstepdotdotfr;
      theta0stepdotdot = theta0stepdotdotfr;
    }
    else{
      xstep = xstepc;
      ystep = ystepc;
      thstep = thstepc;
      theta0step = theta0stepc;

      xstepdot = xstepdotc;
      ystepdot = ystepdotc;
      thstepdot = thstepdotc;
      theta0stepdot = theta0stepdotc;

      xstepdotdot = xstepdotdotc;
      ystepdotdot = ystepdotdotc;
      thstepdotdot = thstepdotdotc;
      theta0stepdotdot = theta0stepdotdotc;
    }

    xstep = xstepfr;
    ystep = ystepfr;
    thstep = thstepfr;
    theta0step = theta0stepfr;

    xstepdot = xstepdotfr;
    ystepdot = ystepdotfr;
    thstepdot = thstepdotfr;
    theta0stepdot = theta0stepdotfr;

    xstepdotdot = xstepdotdotfr;
    ystepdotdot = ystepdotdotfr;
    thstepdotdot = thstepdotdotfr;
    theta0stepdotdot = theta0stepdotdotfr;

    msg_xd_x.data = xstep;
    msg_xd_y.data = ystep;
    msg_xd_theta.data = thstep;


}



void controller(){

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

  /*JE DOT coefficients*/
  double jedot11, jedot12, jedot13, jedot14, jedot15, jedot16;
  double jedot21, jedot22, jedot23, jedot24, jedot25, jedot26;
  double jedot31, jedot32, jedot33, jedot34, jedot35, jedot36;

  Eigen::VectorXd jvw(2);
  Eigen::MatrixXd jvq(2,3);
  Eigen::MatrixXd jac1(6,6);
  Eigen::MatrixXd jac1dot(6,6);
  Eigen::VectorXd cstar(6);
  Eigen::MatrixXd hstar(6,6);
  Eigen::MatrixXd jstar(6,6);
  Eigen::MatrixXd jestar(6,3);

  Eigen::MatrixXd h11star(2,2);
  Eigen::MatrixXd h12star(2,4);
  Eigen::MatrixXd h21star(4,2);
  Eigen::MatrixXd h22star(4,4);

  Eigen::MatrixXd hbar(4,4);

  Eigen::VectorXd c1star(2);
  Eigen::VectorXd c2star(4);
  Eigen::MatrixXd cbar(4,4);

  Eigen::MatrixXd je11star(2,2);
  Eigen::VectorXd je12star(2);
  Eigen::MatrixXd je21star(4,2);
  Eigen::VectorXd je22star(4);

  Eigen::MatrixXd jebar(4,3);
  Eigen::VectorXd qe(3);

  Eigen::VectorXd xdotdot_des(4);
  Eigen::VectorXd fdes(4);
  Eigen::VectorXd v1(6);



  double p1=M;
  double p2=(m1+m2+m3)*r0x;
  double p3=(m1+m2+m3)*r0y;
  double p4=(m1+m2+m3)*l1+(m2+m3)*r1;
  double p5=(m2+m3)*l2+m3*r2;
  double p6=i0zz+(m1+m2+m3)*(r0x*r0x+r0y*r0y);
  double p7=i1zz+(m1+m2+m3)*l1*l1+2*(m2+m3)*l1*r1+(m2+m3)*r1*r1;
  double p8=i2zz+(m2+m3)*l2*l2+2*m3*l2*r2+m3*r2*r2;
  double p9=i3zz+m3*l3*l3;
  double p10=((m1+m2+m3)*l1+(m2+m3)*r1)*r0x;
  double p11=((m1+m2+m3)*l1+(m2+m3)*r1)*r0y;
  double p12=(l1+r1)*((m2+m3)*l2+m3*r2);
  double p13=((m2+m3)*l2+m3*r2)*r0x;
  double p14=((m2+m3)*l2+m3*r2)*r0y;
  double p15=m3*l3;
  double p16=m3*l3*r0x;
  double p17=m3*l3*r0y;
  double p18=(l1+r1)*m3*l3;
  double p19=(l2+r2)*m3*l3;
  double a11 = pow(10,20);
  double a22 = pow(10,-20);
  double z_free=1;
  double ts_f=0.2*t_free;
  double wn_free=6/ts_f;
  double kdf=1;
  double mdf=kdf/pow(wn_free,2);
  double bdf=2*z_free*wn_free*mdf;
  Eigen::MatrixXd md_f = mdf*Eigen::MatrixXd::Identity(4,4);
  Eigen::MatrixXd bd_f = bdf*Eigen::MatrixXd::Identity(4,4);
  Eigen::MatrixXd kd_f = kdf*Eigen::MatrixXd::Identity(4,4);
  double ke = pow(10,6);
  double z_contact=z_free*sqrt(kdf/(kdf+ke));
  double wn_contact=wn_free*sqrt((kdf+ke)/kdf);
  double fd = 0.1;
  double kdc = 100;
  double mdc=(kdc+ke)/pow(wn_contact,2);
  double be = 0;
  double bdc=2*z_contact*wn_contact*mdc-be;
  Eigen::MatrixXd md_c = mdc*Eigen::MatrixXd::Identity(4,4);
  Eigen::MatrixXd bd_c = bdc*Eigen::MatrixXd::Identity(4,4);
  Eigen::MatrixXd kd_c = kdc*Eigen::MatrixXd::Identity(4,4);






  h11 = p1;
  h12 = 0;
  h13 = (-1)*p3*cos(theta0) + (-1)*p2*sin(theta0) + (-1)*p4*sin(q1 + theta0) +(-1)*p5*sin(q1 + q2 + theta0) + (-1)*p15*sin(q1 + q2 + q3 + theta0);
  h14 = (-1)*p4*sin(q1 + theta0) + (-1)*p5*sin(q1 + q2 + theta0) + (-1)*p15*sin(q1 + q2 + q3 + theta0);
  h15 = (-1)*p5*sin(q1 + q2 + theta0) + (-1)*p15*sin(q1 + q2 + q3 + theta0);
  h16 = (-1)*p15*sin(q1 + q2 + q3 + theta0);

  h21 = 0;
  h22 = p1;
  h23 = p2*cos(theta0) + p4*cos(q1 + theta0) + p5*cos(q1 + q2 +theta0) + p15*cos(q1 + q2 + q3 + theta0) + (-1)*p3*sin(theta0);
  h24 = p4*cos(q1 + theta0) +p5*cos(q1 + q2 + theta0) + p15*cos(q1 + q2 + q3 + theta0);
  h25 = p5*cos(q1 + q2 + theta0) + p15*cos(q1 + q2 + q3 + theta0);
  h26 = p15*cos(q1 + q2 + q3 + theta0);

  h31 = (-1)*p3*cos(theta0) + (-1)* p2*sin(theta0)+(-1)*p4*sin(q1 + theta0)+(-1)*p5*sin(q1 + q2 + theta0) + (-1)*p15*sin(q1 + q2 + q3 + theta0);
  h32 = p2*cos(theta0) + p4*cos(q1 + theta0) + p5*cos(q1 + q2 + theta0) + p15*cos(q1 + q2 + q3 + theta0) + (-1)*p3*sin(theta0);
  h33 = p6 + p7 + p8 + p9 + 2*p10*cos(q1) + 2*p12*cos(q2) + 2*p13*cos(q1 + q2) + 2*p19*cos(q3) +2*p18*cos(q2 + q3) + 2*p16*cos(q1 + q2 + q3) + 2*p11*sin(q1) + 2*p14* sin(q1 + q2) + 2*p17*sin(q1 + q2 + q3);
  h34 = p7 + p8 + p9 + p10*cos(q1) + 2*p12*cos(q2)+p13*cos(q1 + q2) + 2*p19*cos(q3) + 2*p18*cos(q2 + q3) + p16* cos(q1 + q2 + q3) + p11*sin(q1) + p14*sin(q1 + q2) + p17*sin(q1 + q2 + q3);
  h35 = p8 + p9 + p12*cos(q2) + p13*cos(q1 + q2) + 2*p19*cos(q3) + p18*cos(q2 + q3) + p16*cos(q1 + q2 + q3) + p14*sin(q1 + q2) + p17*sin(q1 + q2 + q3);
  h36 = p9 + p19*cos(q3) + p18*cos(q2 + q3) + p16*cos(q1 + q2 + q3) + p17*sin(q1 + q2 + q3);

  h41 = (-1)*p4*sin(q1 + theta0) + (-1)*p5*sin(q1 + q2 + theta0) + (-1)*p15*sin(q1 + q2 + q3 + theta0);
  h42 = p4*cos(q1 + theta0) + p5*cos(q1 + q2 + theta0) + p15*cos(q1 + q2 + q3 + theta0);
  h43 = p7 + p8 + p9 + p10*cos(q1) + 2*p12*cos(q2) + p13*cos(q1 + q2) + 2*p19*cos(q3)+ 2*p18*cos(q2 + q3) + p16*cos(q1 + q2 + q3) + p11*sin(q1) + p14*sin(q1 +q2) + p17*sin(q1 + q2 + q3);
  h44 = p7 + p8 + p9 + 2*p12*cos(q2) + 2*p19*cos(q3) + 2*p18*cos(q2 + q3);
  h45 = p8 + p9 + p12*cos(q2) + 2*p19*cos(q3) + p18*cos(q2 + q3);
  h46 = p9 + p19*cos(q3) + p18*cos(q2 + q3);

  h51 = (-1)*p5*sin(q1 + q2 + theta0) + (-1)*p15*sin(q1 + q2 + q3 + theta0);
  h52 = p5*cos(q1 + q2 + theta0) + p15*cos(q1 + q2 + q3 + theta0);
  h53 = p8 + p9 + p12*cos(q2) + p13*cos(q1 + q2) + 2*p19*cos(q3) + p18*cos(q2 +q3) + p16*cos(q1 + q2 + q3) + p14*sin(q1 + q2) + p17*sin(q1 + q2 + q3);
  h54 = p8 + p9 +p12*cos(q2) + 2*p19*cos(q3) + p18*cos(q2 + q3);
  h55 = p8 + p9 + 2*p19*cos(q3);
  h56 = p9 + p19*cos(q3);

  h61 = (-1)*p15*sin(q1 + q2 + q3 + theta0);
  h62 = p15*cos(q1 + q2 + q3 + theta0);
  h63 = p9 + p19*cos(q3) + p18*cos(q2 + q3) + p16*cos(q1 + q2 + q3) + p17*sin(q1 + q2 + q3);
  h64 = p9 + p19*cos(q3) + p18*cos(q2 + q3);
  h65 = p9 + p19*cos(q3);
  h66 = p9;

  c11 =(-1)*p2*pow(theta0dot,2)*cos(theta0) + (-1)*p4*pow((q1dot + theta0dot),2)*cos( 
    q1 + theta0) + (-1)*p5*pow(q1dot,2)*cos(q1 + q2 + theta0) + (-2)*p5*q1dot* 
    q2dot*cos(q1 + q2 + theta0) + (-1)*p5*pow(q2dot,2)*cos(q1 + q2 + theta0) + (-2)* 
    p5*q1dot*theta0dot*cos(q1 + q2 + theta0) + (-2)*p5*q2dot*theta0dot*cos(q1 +  
    q2 + theta0) + (-1)*p5*pow(theta0dot,2)*cos(q1 + q2 + theta0) + (-1)*p15*pow(q1dot,2)* 
    cos(q1 + q2 + q3 + theta0) + (-2)*p15*q1dot*q2dot*cos(q1 + q2 + q3 + theta0) + (-1) 
    *p15*pow(q2dot,2)*cos(q1 + q2 + q3 + theta0) + (-2)*p15*q1dot*q3dot*cos( 
    q1 + q2 + q3 + theta0) + (-2)*p15*q2dot*q3dot*cos(q1 + q2 + q3 + theta0) + (-1)* 
    p15*pow(q3dot,2)*cos(q1 + q2 + q3 + theta0) + (-2)*p15*q1dot*theta0dot*cos(q1 +  
    q2 + q3 + theta0) + (-2)*p15*q2dot*theta0dot*cos(q1 + q2 + q3 + theta0) + (-2)*p15* 
    q3dot*theta0dot*cos(q1 + q2 + q3 + theta0) + (-1)*p15*pow(theta0dot,2)*cos(q1 + q2 + q3 + theta0) + p3*pow(theta0dot,2)*sin(theta0);

  c21 = (-1)*p3*pow(theta0dot,2)*cos(theta0) + (-1) 
    *p2*pow(theta0dot,2)*sin(theta0) + (-1)*p4*pow(q1dot,2)*sin(q1 + theta0) + (-2)* 
    p4*q1dot*theta0dot*sin(q1 + theta0) + (-1)*p4*pow(theta0dot,2)*sin(q1 + theta0) + ( 
    -1)*p5*pow(q1dot,2)*sin(q1 + q2 + theta0) + (-2)*p5*q1dot*q2dot*sin(q1 +  
    q2 + theta0) + (-1)*p5*pow(q2dot,2)*sin(q1 + q2 + theta0) + (-2)*p5*q1dot* 
    theta0dot*sin(q1 + q2 + theta0) + (-2)*p5*q2dot*theta0dot*sin(q1 + q2 + theta0) + ( 
    -1)*p5*pow(theta0dot,2)*sin(q1 + q2 + theta0) + (-1)*p15*pow(q1dot,2)*sin(q1 + q2 +  
    q3 + theta0) + (-2)*p15*q1dot*q2dot*sin(q1 + q2 + q3 + theta0) + (-1)*p15* 
    pow(q2dot,2)*sin(q1 + q2 + q3 + theta0) + (-2)*p15*q1dot*q3dot*sin(q1 + q2 + q3 +  
    theta0) + (-2)*p15*q2dot*q3dot*sin(q1 + q2 + q3 + theta0) + (-1)*p15* 
    pow(q3dot,2)*sin(q1 + q2 + q3 + theta0) + (-2)*p15*q1dot*theta0dot*sin(q1 + q2 +  
    q3 + theta0) + (-2)*p15*q2dot*theta0dot*sin(q1 + q2 + q3 + theta0) + (-2)*p15* 
    q3dot*theta0dot*sin(q1 + q2 + q3 + theta0) + (-1)*p15*pow(theta0dot,2)*sin(q1 + q2 +  
    q3 + theta0);

  c31 = p11*q1dot*(q1dot + 2*theta0dot)*cos(q1) + p14*(q1dot + q2dot) 
    *(q1dot + q2dot + 2*theta0dot)*cos(q1 + q2) + p17*pow(q1dot,2)*cos(q1 + q2 + q3) 
    + 2*p17*q1dot*q2dot*cos(q1 + q2 + q3) + p17*pow(q2dot,2)*cos(q1 + q2 + q3) +  
    2*p17*q1dot*q3dot*cos(q1 + q2 + q3) + 2*p17*q2dot*q3dot*cos(q1 +  
    q2 + q3) + p17*pow(q3dot,2)*cos(q1 + q2 + q3) + 2*p17*q1dot*theta0dot*cos(q1 +  
    q2 + q3) + 2*p17*q2dot*theta0dot*cos(q1 + q2 + q3) + 2*p17*q3dot* 
    theta0dot*cos(q1 + q2 + q3) + (-1)*p10*pow(q1dot,2)*sin(q1) + (-2)*p10* 
    q1dot*theta0dot*sin(q1) + (-2)*p12*q1dot*q2dot*sin(q2) + (-1)* 
    p12*pow(q2dot,2)*sin(q2) + (-2)*p12*q2dot*theta0dot*sin(q2) + (-1)* 
    p13*pow(q1dot,2)*sin(q1 + q2) + (-2)*p13*q1dot*q2dot*sin(q1 + q2) + (-1) 
    *p13*pow(q2dot,2)*sin(q1 + q2) + (-2)*p13*q1dot*theta0dot*sin(q1 + q2) + ( 
    -2)*p13*q2dot*theta0dot*sin(q1 + q2) + (-2)*p19*q1dot*q3dot*sin( 
    q3) + (-2)*p19*q2dot*q3dot*sin(q3) + (-1)*p19*pow(q3dot,2)*sin(q3) +  
    (-2)*p19*q3dot*theta0dot*sin(q3) + (-2)*p18*q1dot*q2dot*sin(q2 +  
    q3) + (-1)*p18*pow(q2dot,2)*sin(q2 + q3) + (-2)*p18*q1dot*q3dot*sin( 
    q2 + q3) + (-2)*p18*q2dot*q3dot*sin(q2 + q3) + (-1)*p18*pow(q3dot,2)* 
    sin(q2 + q3) + (-2)*p18*q2dot*theta0dot*sin(q2 + q3) + (-2)*p18*q3dot* 
    theta0dot*sin(q2 + q3) + (-1)*p16*pow(q1dot,2)*sin(q1 + q2 + q3) + (-2)*p16* 
    q1dot*q2dot*sin(q1 + q2 + q3) + (-1)*p16*pow(q2dot,2)*sin(q1 + q2 + q3) + ( 
    -2)*p16*q1dot*q3dot*sin(q1 + q2 + q3) + (-2)*p16*q2dot*q3dot* 
    sin(q1 + q2 + q3) + (-1)*p16*pow(q3dot,2)*sin(q1 + q2 + q3) + (-2)*p16* 
    q1dot*theta0dot*sin(q1 + q2 + q3) + (-2)*p16*q2dot*theta0dot*sin(q1 + q2 +  
    q3) + (-2)*p16*q3dot*theta0dot*sin(q1 + q2 + q3);

  c41 = (-1)*p11*pow(theta0dot,2)* 
    cos(q1) + (-1)*p14*pow(theta0dot,2)*cos(q1 + q2) + (-1)*p17*pow(theta0dot,2)* 
    cos(q1 + q2 + q3) + p10*pow(theta0dot,2)*sin(q1) + (-2)*p12*q1dot*q2dot* 
    sin(q2) + (-1)*p12*pow(q2dot,2)*sin(q2) + (-2)*p12*q2dot*theta0dot* 
    sin(q2) + p13*pow(theta0dot,2)*sin(q1 + q2) + (-2)*p19*q1dot*q3dot*sin( 
    q3) + (-2)*p19*q2dot*q3dot*sin(q3) + (-1)*p19*pow(q3dot,2)*sin(q3) +  
    (-2)*p19*q3dot*theta0dot*sin(q3) + (-2)*p18*q1dot*q2dot*sin(q2 +  
    q3) + (-1)*p18*pow(q2dot,2)*sin(q2 + q3) + (-2)*p18*q1dot*q3dot*sin( 
    q2 + q3) + (-2)*p18*q2dot*q3dot*sin(q2 + q3) + (-1)*p18*pow(q3dot,2)* 
    sin(q2 + q3) + (-2)*p18*q2dot*theta0dot*sin(q2 + q3) + (-2)*p18*q3dot* 
    theta0dot*sin(q2 + q3) + p16*pow(theta0dot,2)*sin(q1 + q2 + q3);

  c51 = (-1)*p14* 
    pow(theta0dot,2)*cos(q1 + q2) + (-1)*p17*pow(theta0dot,2)*cos(q1 + q2 + q3) + p12* 
    pow(q1dot,2)*sin(q2) + 2*p12*q1dot*theta0dot*sin(q2) + p12*pow(theta0dot,2)* 
    sin(q2) + p13*pow(theta0dot,2)*sin(q1 + q2) + (-2)*p19*q1dot*q3dot*sin( 
    q3) + (-2)*p19*q2dot*q3dot*sin(q3) + (-1)*p19*pow(q3dot,2)*sin(q3) +  
    (-2)*p19*q3dot*theta0dot*sin(q3) + p18*pow(q1dot,2)*sin(q2 + q3) + 2* 
    p18*q1dot*theta0dot*sin(q2 + q3) + p18*pow(theta0dot,2)*sin(q2 + q3) + p16* 
    pow(theta0dot,2)*sin(q1 + q2 + q3);

  c61 = (-1)*p17*pow(theta0dot,2)*cos(q1 + q2 + q3) + p19* 
    pow((q1dot + q2dot + theta0dot),2)*sin(q3) + p18*pow(q1dot,2)*sin(q2 + q3) + 2* 
    p18*q1dot*theta0dot*sin(q2 + q3) + p18*pow(theta0dot,2)*sin(q2 + q3) + p16* 
    pow(theta0dot,2)*sin(q1 + q2 + q3);


  //to theta 1 einai to q1 ousiastika kai to th0 to theta0

  je11 = 1;
  je12 = 0;
  je13 = (-1)*r0y*cos(theta0) + (-1)*r0x*sin(theta0) + (-1)*l1*sin( 
    q01 + theta0 + q1) + (-1)*r1*sin(q01 + theta0 + q1) + (-1)*l2*sin( 
    q01 + theta0 + q1 + q2) + (-1)*r2*sin(q01 + theta0 + q1 + q2) +  
    (-1)*l3*sin(q01 + theta0 + q1 + q2 + q3) + (-1)*r3*sin( q01 + theta0 + q1 + q2 + q3);
  je14 = (-1)*(l1 + r1)*sin(q01 + theta0 +  
    q1) + (-1)*(l2 + r2)*sin(q01 + theta0 + q1 + q2) + (-1)*(l3 +  
    r3)*sin(q01 + theta0 + q1 + q2 + q3);
  je15 = (-1)*(l2 + r2)*sin( 
    q01 + theta0 + q1 + q2) + (-1)*(l3 + r3)*sin(q01 + theta0 +  
    q1 + q2 + q3);
  je16 = (-1)*(l3 + r3)*sin(q01 + theta0 + q1 + q2 + q3);

  je21 = 0;
  je22 = 1;
  je23 = r0x*cos(theta0) + (l1 + r1)*cos(q01 + theta0 +q1) + l2*cos(q01 + theta0 + q1 + q2) + r2*cos(q01 + theta0 +  
    q1 + q2) + l3*cos(q01 + theta0 + q1 + q2 + q3) + r3*cos(q01 + theta0 + q1 + q2 + q3) + (-1)*r0y*sin(theta0);
  je24 = (l1 +  
    r1)*cos(q01 + theta0 + q1) + (l2 + r2)*cos(q01 + theta0 + q1 + q2) 
    + (l3 + r3)*cos(q01 + theta0 + q1 + q2 + q3);
  je25 = (l2 + r2)*cos( 
    q01 + theta0 + q1 + q2) + (l3 + r3)*cos(q01 + theta0 + q1 +  
    q2 + q3);
  je26 = (l3 + r3)*cos(q01 + theta0 + q1 + q2 + q3);

  je31 = 0;
  je32 = 0;
  je33 = 1;
  je34 = 1;
  je35 = 1;
  je36 = 1;

  std::cout<<"theta0 is: "<<theta0<<std::endl;
  std::cout<<"theta0step is: "<<theta0step<<std::endl;
  std::cout<<"theta0dot is: "<<theta0dot<<std::endl; 
  std::cout<<"theta0stepdot is: "<<theta0stepdot<<std::endl;

  std::cout<<"r0x is: "<<r0x<<std::endl;
  std::cout<<"r0y is: "<<r0y<<std::endl;
  std::cout<<"l1 is: "<<l1<<std::endl;
  std::cout<<"l2 is: "<<l2<<std::endl;
  std::cout<<"l3 is: "<<l3<<std::endl;
  std::cout<<"r1 is: "<<r1<<std::endl;
  std::cout<<"r2 is: "<<r2<<std::endl;
  std::cout<<"r3 is: "<<r3<<std::endl;
  std::cout<<"q01 is: "<<q01<<std::endl;
  std::cout<<"q1 is: "<<q1<<std::endl;
  std::cout<<"q2 is: "<<q2<<std::endl;
  std::cout<<"q3 is: "<<q3<<std::endl;
  std::cout<<"q1dot is: "<<q1dot<<std::endl;
  std::cout<<"q2dot is: "<<q2dot<<std::endl;
  std::cout<<"q3dot is: "<<q3dot<<std::endl;
  
 




  jedot11 = 0;
  jedot12 = 0;
  jedot13 = (-1)*r0x*theta0dot*cos(theta0) + (-1)*l1*(theta0dot +  
    q1dot)*cos(q01 + theta0 + q1) + (-1)*r1*(theta0dot + q1dot) 
    *cos(q01 + theta0 + q1) + (-1)*l2*(theta0dot + q1dot + q2dot) 
    *cos(q01 + theta0 + q1 + q2) + (-1)*r2*(theta0dot + q1dot +  
    q2dot)*cos(q01 + theta0 + q1 + q2) + (-1)*l3*(theta0dot +  
    q1dot + q2dot + q3dot)*cos(q01 + theta0 + q1 + q2 +  
    q3) + (-1)*r3*(theta0dot + q1dot + q2dot + q3dot)* 
    cos(q01 + theta0 + q1 + q2 + q3) + r0y*theta0dot*sin(theta0);
  jedot14 = ( 
    -1)*(l1 + r1)*(theta0dot + q1dot)*cos(q01 + theta0 + q1) + (-1)* 
    (l2 + r2)*(theta0dot + q1dot + q2dot)*cos(q01 + theta0 + q1 +  
    q2) + (-1)*(l3 + r3)*(theta0dot + q1dot + q2dot +  
    q3dot)*cos(q01 + theta0 + q1 + q2 + q3);
  jedot15 = (-1)*(l2 + r2)*( 
    theta0dot + q1dot + q2dot)*cos(q01 + theta0 + q1 + q2) + (-1) 
    *(l3 + r3)*(theta0dot + q1dot + q2dot + q3dot)*cos(q01 +  
    theta0 + q1 + q2 + q3);
  jedot16 = (-1)*(l3 + r3)*(theta0dot +  
    q1dot + q2dot + q3dot)*cos(q01 + theta0 + q1 + q2 +  
    q3);

  jedot21 = 0;
  jedot22 = 0;
  jedot23 = (-1)*r0y*theta0dot*cos(theta0) + (-1)*r0x* 
    theta0dot*sin(theta0) + (-1)*(l1 + r1)*(theta0dot + q1dot)*sin( 
    q01 + theta0 + q1) + (-1)*l2*(theta0dot + q1dot + q2dot)*sin( 
    q01 + theta0 + q1 + q2) + (-1)*r2*(theta0dot + q1dot +  
    q2dot)*sin(q01 + theta0 + q1 + q2) + (-1)*l3*(theta0dot +  
    q1dot + q2dot + q3dot)*sin(q01 + theta0 + q1 + q2 +  
    q3) + (-1)*r3*(theta0dot + q1dot + q2dot + q3dot)* 
    sin(q01 + theta0 + q1 + q2 + q3);
  jedot24 = (-1)*(l1 + r1)*(theta0dot +  
    q1dot)*sin(q01 + theta0 + q1) + (-1)*(l2 + r2)*(theta0dot +  
    q1dot + q2dot)*sin(q01 + theta0 + q1 + q2) + (-1)*(l3 +  
    r3)*(theta0dot + q1dot + q2dot + q3dot)*sin(q01 + theta0 +  
    q1 + q2 + q3);
  jedot25 = (-1)*(l2 + r2)*(theta0dot + q1dot +  
    q2dot)*sin(q01 + theta0 + q1 + q2) + (-1)*(l3 + r3)*( 
    theta0dot + q1dot + q2dot + q3dot)*sin(q01 + theta0 + q1 +  
    q2 + q3);
  jedot26 = (-1)*(l3 + r3)*(theta0dot + q1dot + q2dot +  
    q3dot)*sin(q01 + theta0 + q1 + q2 + q3);

  jedot31 = 0;
  jedot32 = 0;
  jedot33 = 0;
  jedot34 = 0;
  jedot35 = 0;
  jedot36 = 0;



  h(0,0) = h11;
  h(0,1) = h12;
  h(0,2) = h13;
  h(0,3) = h14;
  h(0,4) = h15;
  h(0,5) = h16;
  //////////////
  h(1,0) = h21;
  h(1,1) = h22;
  h(1,2) = h23;
  h(1,3) = h24;
  h(1,4) = h25;
  h(1,5) = h26;
  /////////////
  h(2,0) = h31;
  h(2,1) = h32;
  h(2,2) = h33;
  h(2,3) = h34;
  h(2,4) = h35;
  h(2,5) = h36;
  /////////////
  h(3,0) = h41;
  h(3,1) = h42;
  h(3,2) = h43;
  h(3,3) = h44;
  h(3,4) = h45;
  h(3,5) = h46;
  ////////////
  h(4,0) = h51;
  h(4,1) = h52;
  h(4,2) = h53;
  h(4,3) = h54;
  h(4,4) = h55;
  h(4,5) = h56;
  /////////////
  h(5,0) = h61;
  h(5,1) = h62;
  h(5,2) = h63;
  h(5,3) = h64;
  h(5,4) = h65;
  h(5,5) = h66;

  c(0) = c11;
  c(1) = c21;
  c(2) = c31;
  c(3) = c41;
  c(4) = c51;
  c(5) = c61;


  je(0,0) = je11;
  je(0,1) = je12;
  je(0,2) = je13;
  je(0,3) = je14;
  je(0,4) = je15;
  je(0,5) = je16;
      ///////////
  je(1,0) = je21;
  je(1,1) = je22;
  je(1,2) = je23;
  je(1,3) = je24;
  je(1,4) = je25;
  je(1,5) = je26;
      ///////////
  je(2,0) = je31;
  je(2,1) = je32;
  je(2,2) = je33;
  je(2,3) = je34;
  je(2,4) = je35;
  je(2,5) = je36;


  jedot(0,0) = jedot11;
  jedot(0,1) = jedot12;
  jedot(0,2) = jedot13;
  jedot(0,3) = jedot14;
  jedot(0,4) = jedot15;
  jedot(0,5) = jedot16;
  /////////
  jedot(1,0) = jedot21;
  jedot(1,1) = jedot22;
  jedot(1,2) = jedot23;
  jedot(1,3) = jedot24;
  jedot(1,4) = jedot25;
  jedot(1,5) = jedot26;
  /////////
  jedot(2,0) = jedot31;
  jedot(2,1) = jedot32;
  jedot(2,2) = jedot33;
  jedot(2,3) = jedot34;
  jedot(2,4) = jedot35;
  jedot(2,5) = jedot36;

  jvw << je13 , je23;
  //std::cout<<"jvw check "<<std::endl;
  std::cout <<"jvw is: "<<jvw<<std::endl;

  jvq << je14, je15, je16,
          je24, je25, je26;
  //std::cout<<"jvq check "<<std::endl;
  std::cout <<"jvq is: "<<jvq<<std::endl;



  jac1 << 1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0,
          1, 0, je13, je14, je15, je16,
          0, 1, je23, je24, je25, je26,
          0, 0, 1, je34, je35, je36;

  //std::cout<<"jac1 check "<<std::endl;
  std::cout <<"jac1 is: "<<jac1<<std::endl;


  
  jac1dot << 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, jedot13, jedot14, jedot15, jedot16,
        0, 0, jedot23, jedot24, jedot25, jedot26,
        0, 0, 0, jedot34, jedot35, jedot36;

  //std::cout<<"jac1dot check "<<std::endl;
  std::cout <<"jac1dot is: "<<jac1dot<<std::endl;

  // std::cout<<"jac1.transpose().inverse() is: "<<(jac1.transpose()).inverse()<<std::endl;
  std::cout<<"h is: "<<h<<std::endl;
  // std::cout<<"jac1.innverse is: "<<jac1.inverse()<<std::endl;

  // Step 1: Compute A'^-1 (inverse of the transpose of A)
  // Eigen::HouseholderQR<Eigen::MatrixXd> qr(jac1.transpose());
  // Eigen::MatrixXd jac1_inv_T = qr.solve(Eigen::MatrixXd::Identity(6,6)); // A'^-1
  // std::cout<<"jac1_inv_T is: "<<jac1_inv_T<<std::endl;

  // // Step 2: Compute A^-1 using QR decomposition
  // Eigen::HouseholderQR<Eigen::Matrix<double, 6, 6>> qr_A(jac1);
  // Eigen::Matrix<double, 6, 6> jac1_inv = qr_A.solve(Eigen::Matrix<double, 6, 6>::Identity()); // A^-1
  // std::cout<<"jac1_inv is: "<<jac1_inv<<std::endl;

  // Step 3: Compute A'^-1 * B * A^-1
  // hstar = jac1_inv_T * h * jac1_inv;
  
  hstar = (jac1.transpose()).inverse()*h*jac1.inverse();

  

  std::cout <<"hstar is: "<<hstar<<std::endl;


  
  v1 <<xc0dot, yc0dot, theta0dot, q1dot, q2dot, q3dot;
  //std::cout<<"v1 check "<<std::endl;
  std::cout <<"v1 is: "<<v1<<std::endl;


  cstar = (jac1.transpose()).inverse()*(c-h*jac1.inverse()*jac1dot*v1); //na ftiakso to v1
  std::cout<<"cstar is: "<<cstar<<std::endl;
  // v1 = qDot;% = [rbdot; theta0dot; qdot_joint);

  jstar = (jac1.transpose()).inverse();
  std::cout<<"jstar is: "<<jstar<<std::endl;

  Eigen::MatrixXd j12star(2,4);
  Eigen::MatrixXd j22star(4,4);

  j12star << jstar(0,2), jstar(0,3), jstar(0,4), jstar(0,5),
              jstar(1,2), jstar(1,3), jstar(1,4), jstar(1,5);
  //std::cout<<"j12star check "<<std::endl;

  
  j22star << jstar(2,2), jstar(2,3), jstar(2,4), jstar(2,5),
              jstar(3,2), jstar(3,3), jstar(3,4), jstar(3,5),
              jstar(4,2), jstar(4,3), jstar(4,4), jstar(4,5),
              jstar(5,2), jstar(5,3), jstar(5,4), jstar(5,5);

  //std::cout<<"j22star check "<<std::endl;
  std::cout<<"j22star is: "<<j22star<<std::endl;




  jestar = jstar*(je.transpose());
  std::cout<<"jestar is: "<<jestar<<std::endl;


  h11star << hstar(0,0), hstar(0,1),
            hstar(1,0), hstar(1,1);

  //std::cout<<"h11star check "<<std::endl;
  std::cout<<"h11star is: "<<h11star<<std::endl;

  
  h12star << hstar(0,2), hstar(0,3), hstar(0,4), hstar(0,5),
            hstar(1,2), hstar(1,3), hstar(1,4), hstar(1,5);

  //std::cout<<"h12star check "<<std::endl;
  std::cout<<"h12star is: "<<h12star<<std::endl;
  
  h21star << hstar(2,0), hstar(2,1),
              hstar(3,0), hstar(3,1),
              hstar(4,0), hstar(4,1),
              hstar(5,0), hstar(5,1);

  //std::cout<<"h21star check "<<std::endl;
  std::cout<<"h21star is: "<<h21star<<std::endl;

  
  h22star << hstar(2,2), hstar(2,3), hstar(2,4), hstar(2,5),
              hstar(3,2), hstar(3,3), hstar(3,4), hstar(3,5),
              hstar(4,2), hstar(4,3), hstar(4,4), hstar(4,5),
              hstar(5,2), hstar(5,3), hstar(5,4), hstar(5,5);

  //std::cout<<"h22star check "<<std::endl;
  std::cout<<"h22star is: "<<h22star<<std::endl;

  hbar = h22star - h21star*h11star*h12star;
  std::cout<<"hbar is: "<<hbar<<std::endl;


  c1star << cstar(0), cstar(1);
  std::cout<<"c1star is: "<<c1star<<std::endl;

  //std::cout<<"c1star check "<<std::endl;

  c2star << cstar(2), cstar(3), cstar(4), cstar(5);
  std::cout<<"c2star is: "<<c2star<<std::endl;
  //std::cout<<"c2star check "<<std::endl;

  cbar=c2star-h21star*(h11star.inverse())*c1star;
  std::cout<<"cbar is: "<<cbar<<std::endl;

  je11star << jestar(0,0), jestar(0,1),
              jestar(1,0), jestar(1,1);

  //std::cout<<"je11star check "<<std::endl;

  
  je21star << jestar(2,0), jestar(2,1),
              jestar(3,0), jestar(3,1),
              jestar(4,0), jestar(4,1),
              jestar(5,0), jestar(5,1);

  //std::cout<<"je21star check "<<std::endl;

  
  je12star << jestar(0,2), jestar(1,2);
  //std::cout<<"je12star check "<<std::endl;


  je22star << jestar(2,2), jestar(3,2), jestar(4,2), jestar(5,2);
  //std::cout<<"je22star check "<<std::endl;


// Jebar=[Je21star-H21star*inv(H11star)*Je11star Je22star-H21star*inv(H11star)*Je12star);

Eigen::MatrixXd jbar=j22star-h21star*(h11star.inverse())*j12star;
std::cout<<"jabr is: "<<jbar<<std::endl;

jebar << je21star-h21star*(h11star.inverse())*je11star, je22star-h21star*(h11star.inverse())*je12star;
//std::cout<<"jebar check "<<std::endl;
std::cout<<"jebar is: "<<jebar<<std::endl;


// Qext=[0;0;Fext;0); na to ftiakso

qe << 0, fext(0), 0;  //den eimai sigouros gia afto
std::cout<<"qe is: "<<qe<<std::endl;
//std::cout<<"qe check "<<std::endl;


Eigen::MatrixXd kd=kd_f*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+kd_c*(abs(fext(0))/(abs(fext(0))+a22));
std::cout<<"kd is: "<<kd<<std::endl;
//std::cout<<"kd check "<<std::endl;


Eigen::MatrixXd md=md_f*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+md_c*(abs(fext(0))/(abs(fext(0))+a22));
std::cout<<"md is :"<<md<<std::endl;
//std::cout<<"md check "<<std::endl;


Eigen::MatrixXd bd=bd_f*(abs(1-abs(fext(0))/a11)/(1+a11*abs(fext(0))))+bd_c*(abs(fext(0))/(abs(fext(0))+a22));
std::cout<<"bd is: "<<bd<<std::endl;
//std::cout<<"bd check "<<std::endl;



fdes << 0, 0, fd*abs(fext(0))/(abs(fext(0))+a22), 0;
std::cout<<"fes is: "<<fdes<<std::endl;
//std::cout<<"fdes check "<<std::endl;


// xdotdot_des=[theta0dotdot_des;xEdotdot_des;thetaEdotdot_des);
xdotdot_des<< theta0stepdotdot, xstepdotdot, ystepdotdot, thstepdotdot;
//std::cout<<"xdotdot_des check "<<std::endl;


Eigen::VectorXd error(4);
error << (theta0 - theta0step), (ee_x - xstep), (ee_y - ystep), (thetach - thstep);
std::cout<<"error is: "<<error<<std::endl;
//std::cout<<"error check "<<std::endl;

Eigen::VectorXd error_dot(4);
error_dot << (theta0dot - theta0stepdot), (xeedot(0) - xstepdot), (xeedot(1) - ystepdot), (xeedot(2) - thstepdot);
//std::cout<<"error_dot check "<<std::endl;
std::cout<<"error_dot  is: "<<error_dot<<std::endl;


Eigen::VectorXd qext(4);
qext << 0, 0, fext(0), 0;
std::cout<<"qext is: "<<qext<<std::endl;

Eigen::VectorXd u = xdotdot_des+(md.inverse())*(-kd*error-bd*error_dot-qext + fdes); 
//std::cout<<"u check "<<std::endl;
std::cout<<"u is: "<<u<<std::endl;



Eigen::VectorXd qbar=hbar*u+cbar-jebar*qe;
//std::cout<<"qbar check "<<std::endl;
std::cout<<"qbar is "<<qbar<<std::endl;


std::cout<<"jbar is "<<jbar<<std::endl;


Eigen::VectorXd tau = (jbar.inverse())*qbar;
//std::cout<<"tau check "<<std::endl;
std::cout<<"tau is:  "<<tau<<std::endl;


//std::cout<<"rw torque is:  "<<tau(0)<<" Nm. "<< std::endl;
//std::cout<<"q1 torque is:  "<<tau(1)<<" Nm. "<< std::endl;
//std::cout<<"q2 torque is:  "<<tau(2)<<" Nm. "<< std::endl;
//std::cout<<"q3 torque is:  "<<tau(3)<<" Nm. "<< std::endl;





// tau = tau/100;
double errorth0, errorth0dot;

errorth0 = theta0step -theta0;
errorth0dot = theta0stepdot - theta0dot;
double ns = kprop*errorth0 + kder*errorth0dot;

base_wrench.force.x = 0;  //fx;
base_wrench.force.y = 0;  //fy;
base_wrench.torque.z = tau(0);//ns;

// msg_RW.data = tau(0); 
msg_RW.data = ns;
msg_LS.data = tau(1);
msg_LE.data = tau(2);
msg_LW.data = tau(3);


std::cout<<"/////////////////"<<std::endl;
}