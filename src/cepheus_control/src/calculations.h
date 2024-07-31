#include "includes.h"
#include "variables.h"


/////////////// CALCULATION FUNCTIONS DEFINITION START////////////////////////
/*
void inverseKinematics(){
    double a_cap, b_cap;
    double q1nom, q1denom;
    double q2nom, q2denomElbowUp, q2denomElbowDown;
    a_cap = c*sin(q2);
    b_cap = b + c*cos(q2);
    q2nom = (pow( ee_x_des - a*cos(theta0) - d*cos(thetaee),2) + (ee_y_des - a*sin(theta0) - d*sin(thetaee))^2 - b^2 - c^2)/(2*b*c);
    q2denomElbowUp = sqrt(1-q2nom^2);
    q2denomElbowDown = -sqrt(1-q2nom^2);

    q2des = atan2(q2nom,q2denomElbowDown); //GIA ELBOW DOWN LYSH PAPAP

    q1nom = -( a_cap*(ee_x_des-a*cos(theta0)-d*cos(thetaee))-b_cap*(ee_y_des-a*sin(theta0)-d*sin(thetaee))/(b^2+c^2+2*b*c*cos(q2des)) );
    q1denom = ( a_cap*(ee_y_des-a*sin(theta0)-d*sin(thetaee))-b_cap*(ee_y_des-a*cos(theta0)-d*cos(thetaee))/(b^2+c^2+2*b*c*cos(q2des)) );
    q1des = atan2(q1nom,q1denom)-theta0;
    q3des = thetaee-theta0-q1des-q2des;
}
*/

void initialiseParameters(){//initialise constant parameters
    // l0 = 0.01; 
    // m0 = 13.3; // OXIapo pinaka 5.1 impedance thesis(not anymore 400), tora apo peleq
    // r0x = 1;
    // r0y = 1;
    // r0 = 1;
    // m1 = 0.083;
    // r1 = 1;
    // l1 = 0.13; //1; ta allaksa, ta phga apo xacro tou peleq
    // m2 = 0.187;
    // r2 = 1;
    // l2 = 0.13; //1;
    // m3 = 0.03547;
    // r3 = 0.5;
    // l3 = 0.085; //0.5;
    // mt = 10; //anti gia 100
    // ibzz=(1/2)*m0*(r0x*r0x+r0y*r0y);
    // i1zz=(1/12)*m1*pow((l1+r1),2);
    // i2zz=(1/12)*m2*pow((l2+r2),2);
    // i3zz=(1/12)*m3*pow((l3+r3),2);
    //DOKIMI ALLA DEN DOYLEPSE
    // itzz = 0.067; //itzz=(1/6)*mt*(lt^2);
    // ibzz = 0.160875;
    // i1zz = 0.000346;
    // i2zz = 0.000346;
    // i3zz = 0.091927;
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
    

    /* METAVLHTES APO PEIRAMA KOSTA */ 
    // m0=53;  
    // m1=0.2314;
    // m2=0.1;
    // m3=0.046;
    // mt = 1;

    // M=m0+m1+m2+m3;

    // l1=0.185;
    // l2=0.143;
    // l3=0.0411;
    // lt=0.03; //what is this
    // r0x=0.1425;
    // r0y=-0.08225;
    // r1=0.185;
    // r2=0.143;
    // r3=0.0261;
    // l0 = 0;
    // a=sqrt(r0x*r0x+r0y*r0y);
    // b=l1+r1;
    // c=l2+r2;
    // d=l3+r3; //den nomizo na ta xreiazomaste
    
    // ibzz=2.1837;
    // i1zz=6.81*pow(10,-3);
    // i2zz=1.487*pow(10,-5);
    // i3zz=1.2287*pow(10,-5);
    // itzz=1;
    /*ta mhkh prokyptoun apo typous gia afthereta plath kai ypsh (des xacro)*/
    // ibzz = 1.06;
    // i1zz = 0.0026476;
    // i2zz = 0.0006849;
    // i3zz = 0.0000188;
    // itzz = 0.00135;

    /*apo nikiforo 12/7/24*/
    // m0 = 53;
    // l0 = 0;
    // r0x=0.1425;
    // r0y=-0.08225;
    // m1 = 1;
    // l1 = r1 = 0.25;
    // m2 = 0.5;
    // l2 = r2 = 0.25;
    // m3 = 0.1;
    // l3 = r3 = 0.125/2;
    // M=m0+m1+m2+m3;

    // ibzz = (1/2)*m0*radius*radius;
    // i1zz = (1/12)*m1*(x1*x1 + y*y);
    // i2zz = (1/12)*m2*(x2*x2 + y*y);
    // i3zz = (1/12)*m3*(x3*x3 + y*y);


    /*gia peirama me kosta 25/7/24*/
    m0 = 53;
    l0 = 0;
    r0x=0.1425;
    r0y=-0.08225;
    m1 = 0.2314;
    l1 = r1 = 0.185;
    m2 = 0.1;
    l2 = r2 = 0.143;
    m3 = 0.046;
    l3 = r3 = 0.0411;
    mt = 1;
    q01 = -0.5236;
    s01 = 0.5;
    s02 = 0.2;

    ibzz = 2.1837;
    i1zz = 0.0068;
    i2zz = 1.487/100000;
    i3zz = 1.2287/100000;



    double x1,x2,x3;
    double radius = 0.2;
    double y = 0.02;
    x1 = l1+r1;
    x2 = l2+r2;
    x3 = l3+r3;

    M=m0+m1+m2+m3;






    ROS_INFO("[initParams]: Double parameters have been set. \n");
    
    // itzz = 0.067; //allo afto oxi tou kosta


    // M = m0 + m1 + m2 + m3; //1 apo to reaction wheel
    // a = r0*m0/M;
    // b = (l1*m0 + (m0+m1)*r1)/M;
    // c = (l2*(m0 + m1) + (m0 + m1 + m2)*r2)/M;
    //  d = r3 + l3*(m0 + m1 + m2)/M;

  /*//ta arxikopoihsa sto variables.h
    xch_c << 0, 0, 0;
    // xdf << 0, 0, 0;
    // xdc << 0, 0, 0;
    // xee << 0, 0, 0;
    // xeedot << 0, 0, 0;
    e << 0, 0, 0, 0, 0, 0;
    edot << 0, 0, 0, 0, 0, 0;

    fext    << 0, 0, 0;
    z       << 0, 0, 0, 0, 0, 0; //xc0,yx0,theta0,q1,q2,q3
    zdot    << 0, 0, 0 ,0 ,0 ,0;
    jacobian << 1, 0, 0, 0, 0, 0, 
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 1, 1, 1;

    jacobiandot <<  1, 0, 0, 0, 0, 0, 
                    0, 1, 0, 0, 0, 0,
                    0, 0, 1, 1, 1, 1;

    c << 0, 0, 0, 0, 0, 0;

    h << 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0;      
    
    rEddotdot << 0, 0, 0;

    je << 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0;

    jb << 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0;
    
    jbdot << 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0;
    */ //ta arxikopoihsa sto variables.h


    // z_contact = z_free*sqrt(kd(0,0)/(kd(0,0)+ke_star(0,0))); //einai pali den ta thelo
    // wn_contact = wn_free*sqrt(kd(0,0)/(kd(0,0)+ke_star(0,0)));
    
    // kd_e << 1000, 0, 0,
    //         0, 1000, 0,
    //         0, 0, 1000;

    // kd_b << 100, 0, 0,
    //         0, 100, 0,
    //         0, 0, 100;

    // md_e << kd_e(0,0)/(wn_free*wn_free), 0, 0,
    //         0, kd_e(1,1)/(wn_free*wn_free), 0,
    //         0, 0, kd_e(2,2)/(wn_free*wn_free);

    // md_e = 30000*i3; //i3:identity(3,3)
    // md_b = 30000*i3;
    // md.topLeftCorner(3,3) = md_e;
    // md.topRightCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    // md.bottomLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    // md.bottomRightCorner(3,3) = md_b;
    //anagastika me to xeri giati olo xalaei
    // c << 0, 0, 0, 0, 0, 0;
    // z << 0, 0, 0, 0, 0, 0;
    // zdot << 0, 0, 0, 0, 0, 0;
    // zddotdot << 0, 0, 0, 0, 0, 0;
    // e << 0, 0, 0, 0, 0, 0;
    // edot << 0, 0, 0, 0, 0, 0;
    // edotdot << 0, 0, 0, 0, 0, 0;
    a_x << 0, 0, 0, 0, 0, 0;
    a_y << 0, 0, 0, 0, 0, 0;
    a_theta << 0, 0, 0, 0, 0, 0;
    // qact << 0, 0, 0, 0, 0, 0;

    md << md_e(0,0) , md_e(0,1), md_e(0,2) , 0, 0, 0,
          md_e(1,0), md_e(1,1), md_e(1,2) , 0, 0, 0,
          md_e(2,0), md_e(2,1), md_e(2,2), 0, 0, 0,
          0         , 0        , 0       ,md_b(0,0), md_b(0,1), md_b(0,2),
          0         , 0        , 0       ,md_b(1,0), md_b(1,1), md_b(1,2),
          0         , 0        , 0       ,md_b(2,0), md_b(2,1), md_b(2,2);
    
    ROS_INFO("[initParams]: Md has been set. \n");



    
    bd_e = 2*z_cont*wn_cont*md_e -i3; //anti gia Be ebala i3

    bd_b = 2*z_cont*wn_cont*md_b;
    // bd.topLeftCorner(3,3) = bd_e;
    // bd.topRightCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    // bd.bottomLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    // bd.bottomRightCorner(3,3) = bd_b;

    bd << bd_e(0,0) , bd_e(0,1), bd_e(0,2) , 0, 0, 0,
          bd_e(1,0), bd_e(1,1), bd_e(1,2) , 0, 0, 0,
          bd_e(2,0), bd_e(2,1), bd_e(2,2), 0, 0, 0,
          0         , 0        , 0       ,bd_b(0,0), bd_b(0,1), bd_b(0,2),
          0         , 0        , 0       ,bd_b(1,0), bd_b(1,1), bd_b(1,2),
          0         , 0        , 0       ,bd_b(2,0), bd_b(2,1), bd_b(2,2);

    ROS_INFO("[initParams]: Bd has been set. \n");

    kd_e = wn_cont*wn_cont*md_e - ke_star;
    kd_b = wn_cont*wn_cont*md_b; 
    // kd.topLeftCorner(3,3) = kd_e;
    // kd.topRightCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    // kd.bottomLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    // kd.bottomRightCorner(3,3) = kd_b;

    kd << kd_e(0,0) , kd_e(0,1), kd_e(0,2) , 0, 0, 0,
          kd_e(1,0), kd_e(1,1), kd_e(1,2) , 0, 0, 0,
          kd_e(2,0), kd_e(2,1), kd_e(2,2), 0, 0, 0,
          0         , 0        , 0       ,kd_b(0,0), kd_b(0,1), kd_b(0,2),
          0         , 0        , 0       ,kd_b(1,0), kd_b(1,1), kd_b(1,2),
          0         , 0        , 0       ,kd_b(2,0), kd_b(2,1), kd_b(2,2); 

    ROS_INFO("[initParams]: Kd has been set. \n"); 

    // jb.leftCols(3) = Eigen::MatrixXd::Identity(3,3);
    // jb.rightCols(3) = Eigen::MatrixXd::Zero(3,3);
    jb << 1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0;

    // jbdot = Eigen::MatrixXd::Zero(3,6);


    fdes << 100, 0, 0; //apo ta liga poy einai edo


    v =  0.01*(fdes(0)*z_cont)/(md_e(0,0)*wn_cont); 
    ROS_INFO("[initParams]:v has been set. \n");
    
    // xd << 0, 0, 0; //ta arxikopoihsa sto variables.h
    // xfd << 0, 0, 0;
    // xcd << 0, 0, 0;

    // v =  (fdes(0)*z_contact)/(md_e(0,0)*wn_contact);  einai palio den to theloume

    sdotfin_x = v/(xE_contact-xE_in);
    
    //mono afta ta arxikopoio edo
    b1_x << sin_x, sdotin_x, sdotdotin_x, sfin_x, sdotfin_x, sdotdotfin_x;
    b1_y <<sin_y, sdotin_y, sdotdotin_y, sfin_y, sdotfin_y, sdotdotfin_y;
    b1_theta << sin_theta, sdotin_theta, sdotdotin_theta, sfin_theta, sdotfin_theta, sdotdotfin_theta;

    /*Ta efera apo desired trajectory, afou ypologizontai mono mia fora*/
    // a_matrix << 1, t0, pow(t0,2), pow(t0,3), pow(t0,4), pow(t0,5),
    //             0, 1, 2*t0, 3*pow(t0,2), 4*pow(t0,3), 5*pow(t0,4),
    //             0, 0, 2, 6*t0, 12*pow(t0,2), 20*pow(t0,3),
    //             1, t_free, pow(t_free,2), pow(t_free,3), pow(t_free,4), pow(t_free,5),
    //             0, 1, 2*t_free, 3*pow(t_free,2), 4*pow(t_free,3), 5*pow(t_free,4),
    //             0, 0, 2, 6*t_free, 12*pow(t_free,2), 20*pow(t_free,3) ;

    a_matrix(0,0) = 1;
    a_matrix(0,1) = t0;
    a_matrix(0,2) = pow(t0,2);
    a_matrix(0,3) = pow(t0,3);
    a_matrix(0,4) = pow(t0,4);
    a_matrix(0,5) = pow(t0,5);
    ///////
    a_matrix(1,0) = 0;
    a_matrix(1,1) = 1;
    a_matrix(1,2) = 2*t0;
    a_matrix(1,3) = 3*pow(t0,2);
    a_matrix(1,4) = 4*pow(t0,3);
    a_matrix(1,5) = 5*pow(t0,4);
    ///////
    a_matrix(2,0) = 0;
    a_matrix(2,1) = 0;
    a_matrix(2,2) = 2;
    a_matrix(2,3) = 6*t0;
    a_matrix(2,4) = 12*pow(t0,2);
    a_matrix(2,5) = 20*pow(t0,3);
    //////
    a_matrix(3,0) = 1;
    a_matrix(3,1) = t_free;
    a_matrix(3,2) = pow(t_free,2);
    a_matrix(3,3) = pow(t_free,3);
    a_matrix(3,4) = pow(t_free,4);
    a_matrix(3,5) = pow(t_free,5);
    //////
    a_matrix(4,0) = 0;
    a_matrix(4,1) = 1;
    a_matrix(4,2) = 2*t_free;
    a_matrix(4,3) = 3*pow(t_free,2);
    a_matrix(4,4) = 4*pow(t_free,3);
    a_matrix(4,5) = 5*pow(t_free,4);
    ////////
    a_matrix(5,0) = 0;
    a_matrix(5,1) = 0;
    a_matrix(5,2) = 2;
    a_matrix(5,3) = 6*t_free;
    a_matrix(5,4) = 12*pow(t_free,2);
    a_matrix(5,5) = 20*pow(t_free,3);

    a_x = a_matrix.inverse()*b1_x; //computeInverseAndDetWithCheck()
    a_y = a_matrix.inverse()*b1_y;
    a_theta = a_matrix.inverse()*b1_theta;
    a0x = a_x(0);
    a1x = a_x(1);
    a2x = a_x(2);
    a3x = a_x(3);
    a4x = a_x(4);
    a5x = a_x(5);
    a0y = a_y(0);
    a1y = a_y(1);
    a2y = a_y(2);
    a3y = a_y(3);
    a4y = a_y(4);
    a5y = a_y(5);
    a0t = a_theta(0);
    a1t = a_theta(1);
    a2t = a_theta(2);
    a3t = a_theta(3);
    a4t = a_theta(4);
    a5t = a_theta(5);
    /*telos prosthikis apo desired trajectory*/
}

void calculateMatrices(){  //calculate stuff in each iteration
    //nikiforos we dont use
    // j16 = -l3*sin(theta0+q1+q2+q3);
    // j15 = j16 - l2*sin(theta0+q1+q2);
    // j14 = j15 - l1*sin(theta0+q1);
    // j13 = j14 - l0*sin(theta0);

    // j26 = l3*cos(theta0+q1+q2+q3);
    // j25 = j26 + l2*cos(theta0+q1+q2);
    // j24 = j25 + l1*cos(theta0+q1);
    // j23 = j24 + l0*cos(theta0);

    // jacobian(0,2) = j13;
    // jacobian(0,3) = j14;
    // jacobian(0,4) = j15;
    // jacobian(0,5) = j16;
    // jacobian(1,2) = j23;
    // jacobian(1,3) = j24;
    // jacobian(1,4) = j25;
    // jacobian(1,5) = j26;

    //nikiforos jacobian we dont use
    // j16dot = -l3*cos(theta0+q1+q2+q3)*(theta0dot+q1dot+q2dot+q3dot);
    // j15dot = j16dot - l2*cos(theta0+q1+q2)*(theta0dot+q1dot+q2dot);
    // j14dot = j15dot - l1*cos(theta0+q1)*(theta0dot+q1dot);
    // j13dot = j14dot - l0*cos(theta0)*theta0dot;

    // j26dot = -l3*sin(theta0+q1+q2+q3)*(theta0dot+q1dot+q2dot+q3dot);
    // j25dot = j26dot - l2*sin(theta0+q1+q2)*(theta0dot+q1dot+q2dot);
    // j24dot = j25dot - l1*sin(theta0+q1)*(theta0dot+q1dot);
    // j23dot = j24dot - l0*sin(theta0)*theta0dot;
    
    // jacobiandot(0,2) = j13dot;
    // jacobiandot(0,3) = j14dot;
    // jacobiandot(0,4) = j15dot;
    // jacobiandot(0,5) = j16dot;
    // jacobiandot(1,2) = j23dot;
    // jacobiandot(1,3) = j24dot;
    // jacobiandot(1,4) = j25dot;
    // jacobiandot(1,5) = j26dot;


    je11 = 1;
    je12 = 0;
    je13 = (-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0);
    je14 = (-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0);
    je15 = (-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0);
    je16 = (-1)*l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0);
    ////////////
    je21 = 0;
    je22 = 1;
    je23 = r0x*cos(theta0)+l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0)+r3*cos(q1+q2+q3+theta0)+(-1)*r0y*sin(theta0);
    je24 = l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0)+r3*cos(q1+q2+q3+theta0);
    je25 = l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0)+r3*cos(q1+q2+q3+theta0);
    je26 = l3*cos(q1+q2+q3+theta0)+r3*cos(q1+q2+q3+theta0);
    ///////////
    je31 = 0;
    je32 = 0;
    je33 = 1;
    je34 = 1;
    je35 = 1;
    je36 = 1;
    ///////////
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

    jedot11 = 0;
    jedot12 = 0;
    jedot13 = q3dot*((-1)*l3*cos(q1+q2+q3+theta0)+(-1)*r3*cos(q1+q2+q3+theta0))+ 
              q2dot*((-1)*l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)* 
              l3*cos(q1+q2+q3+theta0)+(-1)*r3*cos(q1+q2+q3+theta0))+q1dot*((-1)* 
              l1*cos(q1+theta0)+(-1)*r1*cos(q1+theta0)+(-1)*l2*cos(q1+q2+theta0)+( 
              -1)*r2*cos(q1+q2+theta0)+(-1)*l3*cos(q1+q2+q3+theta0)+(-1)*r3*cos( 
              q1+q2+q3+theta0))+theta0dot*((-1)*r0x*cos(theta0)+(-1)*l1*cos(q1+theta0)+ 
              (-1)*r1*cos(q1+theta0)+(-1)*l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+ 
              q2+theta0)+(-1)*l3*cos(q1+q2+q3+theta0)+(-1)*r3*cos(q1+q2+q3+theta0)+ 
              r0y*sin(theta0));
    jedot14 = q3dot*((-1)*l3*cos(q1+q2+q3+theta0)+(-1)*r3*cos(q1+q2+q3+theta0))+ 
              q2dot*((-1)*l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)* 
              l3*cos(q1+q2+q3+theta0)+(-1)*r3*cos(q1+q2+q3+theta0))+q1dot*((-1)* 
              l1*cos(q1+theta0)+(-1)*r1*cos(q1+theta0)+(-1)*l2*cos(q1+q2+theta0)+( 
              -1)*r2*cos(q1+q2+theta0)+(-1)*l3*cos(q1+q2+q3+theta0)+(-1)*r3*cos( 
              q1+q2+q3+theta0))+theta0dot*((-1)*l1*cos(q1+theta0)+(-1)*r1*cos(q1+ 
              theta0)+(-1)*l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)*l3* 
              cos(q1+q2+q3+theta0)+(-1)*r3*cos(q1+q2+q3+theta0));
    jedot15 = q3dot*((-1)*l3*cos(q1+q2+q3+theta0)+(-1)*r3*cos(q1+q2+q3+theta0))+ 
              q1dot*((-1)*l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)* 
              l3*cos(q1+q2+q3+theta0)+(-1)*r3*cos(q1+q2+q3+theta0))+q2dot*((-1)* 
              l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)*l3*cos(q1+q2+ 
              q3+theta0)+(-1)*r3*cos(q1+q2+q3+theta0))+theta0dot*((-1)*l2*cos(q1+q2+ 
              theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)*l3*cos(q1+q2+q3+theta0)+(-1)* 
              r3*cos(q1+q2+q3+theta0));
    jedot16 = q1dot*((-1)*l3*cos(q1+q2+q3+theta0)+(-1)*r3*cos(q1+q2+q3+theta0))+ 
              q2dot*((-1)*l3*cos(q1+q2+q3+theta0)+(-1)*r3*cos(q1+q2+q3+theta0))+ 
              q3dot*((-1)*l3*cos(q1+q2+q3+theta0)+(-1)*r3*cos(q1+q2+q3+theta0))+ 
              theta0dot*((-1)*l3*cos(q1+q2+q3+theta0)+(-1)*r3*cos(q1+q2+q3+theta0));
    ////////
    jedot21 = 0;
    jedot22 = 0;
    jedot23 = q3dot*((-1)*l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0))+ 
              q2dot*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)* 
              l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0))+q1dot*((-1)* 
              l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+( 
              -1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin( 
              q1+q2+q3+theta0))+theta0dot*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+( 
              -1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+ 
              theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0)+(-1)* 
              r3*sin(q1+q2+q3+theta0));
    jedot24 = q3dot*((-1)*l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0))+ 
              q2dot*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)* 
              l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0))+q1dot*((-1)* 
              l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+( 
              -1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin( 
              q1+q2+q3+theta0))+theta0dot*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+ 
              theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3* 
              sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0));
    jedot25 = q3dot*((-1)*l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0))+ 
              q1dot*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)* 
              l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0))+q2dot*((-1)* 
              l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+ 
              q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0))+theta0dot*((-1)*l2*sin(q1+q2+ 
              theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0)+(-1)* 
              r3*sin(q1+q2+q3+theta0));
    jedot26 = q1dot*((-1)*l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0))+ 
              q2dot*((-1)*l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0))+ 
              q3dot*((-1)*l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0))+ 
              theta0dot*((-1)*l3*sin(q1+q2+q3+theta0)+(-1)*r3*sin(q1+q2+q3+theta0));
    ////////
    jedot31 = 0;
    jedot32 = 0;
    jedot33 = 0;
    jedot34 = 0;
    jedot35 = 0;
    jedot36 = 0;

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
 


    // jacobian.topRows(3) = je;
    // jacobian.bottomRows(3) = jb;
    // jacobiandot.topRows(3) = jedot;
    // jacobiandot.bottomRows(3) = jbdot;

    jacobian(0,0) = je(0,0);
    jacobian(1,0) = je(1,0);
    jacobian(2,0) = je(2,0);
    jacobian(3,0) = jb(0,0);
    jacobian(4,0) = jb(1,0);
    jacobian(5,0) = jb(2,0);
    ///////////// 
    jacobian(0,1) = je(0,1);
    jacobian(1,1) = je(1,1);
    jacobian(2,1) = je(2,1);
    jacobian(3,1) = jb(0,1);
    jacobian(4,1) = jb(1,1);
    jacobian(5,1) = jb(2,1);
    ////////////
    jacobian(0,2) = je(0,2);
    jacobian(1,2) = je(1,2);
    jacobian(2,2) = je(2,2);
    jacobian(3,2) = jb(0,2);
    jacobian(4,2) = jb(1,2);
    jacobian(5,2) = jb(2,2);
    ///////////// 
    jacobian(0,3) = je(0,3);
    jacobian(1,3) = je(1,3);
    jacobian(2,3) = je(2,3);
    jacobian(3,3) = jb(0,3);
    jacobian(4,3) = jb(1,3);
    jacobian(5,3) = jb(2,3);
    ///////////// 
    jacobian(0,4) = je(0,4);
    jacobian(1,4) = je(1,4);
    jacobian(2,4) = je(2,4);
    jacobian(3,4) = jb(0,4);
    jacobian(4,4) = jb(1,4);
    jacobian(5,4) = jb(2,4);
    ///////////// 
    jacobian(0,5) = je(0,5);
    jacobian(1,5) = je(1,5);
    jacobian(2,5) = je(2,5);
    jacobian(3,5) = jb(0,5);
    jacobian(4,5) = jb(1,5);
    jacobian(5,5) = jb(2,5);

    jacobiandot(0,0) = jedot(0,0);
    jacobiandot(1,0) = jedot(1,0);
    jacobiandot(2,0) = jedot(2,0);
    jacobiandot(3,0) = jbdot(0,0);
    jacobiandot(4,0) = jbdot(1,0);
    jacobiandot(5,0) = jbdot(2,0);
    ///////////// 
    jacobiandot(0,1) = jedot(0,1);
    jacobiandot(1,1) = jedot(1,1);
    jacobiandot(2,1) = jedot(2,1);
    jacobiandot(3,1) = jbdot(0,1);
    jacobiandot(4,1) = jbdot(1,1);
    jacobiandot(5,1) = jbdot(2,1);
    ////////////
    jacobiandot(0,2) = jedot(0,2);
    jacobiandot(1,2) = jedot(1,2);
    jacobiandot(2,2) = jedot(2,2);
    jacobiandot(3,2) = jbdot(0,2);
    jacobiandot(4,2) = jbdot(1,2);
    jacobiandot(5,2) = jbdot(2,2);
    ///////////// 
    jacobiandot(0,3) = jedot(0,3);
    jacobiandot(1,3) = jedot(1,3);
    jacobiandot(2,3) = jedot(2,3);
    jacobiandot(3,3) = jbdot(0,3);
    jacobiandot(4,3) = jbdot(1,3);
    jacobiandot(5,3) = jbdot(2,3);
    ///////////// 
    jacobiandot(0,4) = jedot(0,4);
    jacobiandot(1,4) = jedot(1,4);
    jacobiandot(2,4) = jedot(2,4);
    jacobiandot(3,4) = jbdot(0,4);
    jacobiandot(4,4) = jbdot(1,4);
    jacobiandot(5,4) = jbdot(2,4);
    ///////////// 
    jacobiandot(0,5) = jedot(0,5);
    jacobiandot(1,5) = jedot(1,5);
    jacobiandot(2,5) = jedot(2,5);
    jacobiandot(3,5) = jbdot(0,5);
    jacobiandot(4,5) = jbdot(1,5);
    jacobiandot(5,5) = jbdot(2,5);
    

    

    

    z(0) = xc0;
    z(1) = yc0;
    z(2) = theta0;
    z(3) = q1;
    z(4) = q2;
    z(5) = q3;



    
        
    c11 =    l3*m3*q3dot*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0)+q2dot*(l2*m2*((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*cos(q1+q2+theta0)+m3*(((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0)))+q1dot*(l1*m1*((-1)*q1dot+(-1)*theta0dot)*cos(q1+theta0)+m2*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+l2*((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*cos(q1+q2+theta0))+m3*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0)))+theta0dot*(m1*(l1*((-1)*q1dot+(-1)*theta0dot)*cos(q1+theta0)+(-1)*theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))+m2*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+l2*((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*cos(q1+q2+theta0)+(-1)*theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))+m3*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0)+(-1)*theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0))));

    c21 =    (-1)*l3*m3*q3dot*(q1dot+q2dot+q3dot+theta0dot)*sin(q1+q2+q3+theta0)+q2dot*((-1)*l2*m2*(q1dot+q2dot+theta0dot)*sin(q1+q2+theta0)+m3*((q1dot+q2dot+theta0dot)*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+theta0dot)*sin(q1+q2+q3+theta0)))+q1dot*((-1)*l1*m1*(q1dot+theta0dot)*sin(q1+theta0)+m2*((q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+(-1)*l2*(q1dot+q2dot+theta0dot)*sin(q1+q2+theta0))+m3*((q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+(q1dot+q2dot+theta0dot)*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+theta0dot)*sin(q1+q2+q3+theta0)))+theta0dot*(m1*(theta0dot*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0))+(-1)*l1*(q1dot+theta0dot)*sin(q1+theta0))+m2*(theta0dot*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0))+(q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+(-1)*l2*(q1dot+q2dot+theta0dot)*sin(q1+q2+theta0))+m3*(theta0dot*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0))+(q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+(q1dot+q2dot+theta0dot)*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+theta0dot)*sin(q1+q2+q3+theta0)));

    c31 =    (-1)*xc0dot*(m1*(l1*((-1)*q1dot+(-1)*theta0dot)*cos(q1+theta0)+( 
      -1)*theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))+m2*(((-1)* 
      q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+l2*((-1)* 
      q1dot+(-1)*q2dot+(-1)*theta0dot)*cos(q1+q2+theta0)+(-1)*theta0dot*( 
      r0x*cos(theta0)+(-1)*r0y*sin(theta0)))+m3*(((-1)*q1dot+(-1)* 
      theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+((-1)*q1dot+(-1)* 
      q2dot+(-1)*theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*( 
      (-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+ 
      theta0)+(-1)*theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0))))+(-1)* 
      yc0dot*(m1*(theta0dot*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0))+( 
      -1)*l1*(q1dot+theta0dot)*sin(q1+theta0))+m2*(theta0dot*((-1)*r0y* 
      cos(theta0)+(-1)*r0x*sin(theta0))+(q1dot+theta0dot)*((-1)*l1*sin(q1+ 
      theta0)+(-1)*r1*sin(q1+theta0))+(-1)*l2*(q1dot+q2dot+theta0dot)*sin( 
      q1+q2+theta0))+m3*(theta0dot*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)) 
      +(q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+( 
      q1dot+q2dot+theta0dot)*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+ 
      q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+theta0dot)*sin(q1+q2+q3+theta0))) 
      +q3dot*((-1)*l3*m3*xc0dot*cos(q1+q2+q3+theta0)+(-1)*l3*m3* 
      yc0dot*sin(q1+q2+q3+theta0)+(1/2)*m3*((-2)*l3*(q1dot+q2dot+q3dot+ 
      theta0dot)*(r0x*cos(theta0)+l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos( 
      q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0)+(-1)*r0y* 
      sin(theta0))*sin(q1+q2+q3+theta0)+(-2)*l3*((q1dot+theta0dot)*(l1*cos( 
      q1+theta0)+r1*cos(q1+theta0))+(q1dot+q2dot+theta0dot)*(l2*cos(q1+q2+theta0) 
      +r2*cos(q1+q2+theta0))+l3*(q1dot+q2dot+q3dot+theta0dot)*cos(q1+q2+q3+ 
      theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*sin(q1+q2+q3+ 
      theta0)+2*l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)* 
      cos(q1+q2+q3+theta0)*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)* 
      l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+( 
      -1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))+(-2)*l3* 
      cos(q1+q2+q3+theta0)*((-1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+(( 
      -1)*q1dot+(-1)*theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1) 
      *q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+ 
      q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)* 
      sin(q1+q2+q3+theta0))))+(1/2)*((-1)*m1*(2*(l1*((-1)*q1dot+(-1) 
      *theta0dot)*cos(q1+theta0)+(-1)*theta0dot*(r0x*cos(theta0)+(-1)*r0y* 
      sin(theta0)))*((-1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+l1*((-1) 
      *q1dot+(-1)*theta0dot)*sin(q1+theta0))+2*(l1*(q1dot+theta0dot)*cos( 
      q1+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*(theta0dot*(( 
      -1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0))+(-1)*l1*(q1dot+theta0dot)* 
      sin(q1+theta0)))+(-1)*m2*(2*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos( 
      q1+theta0)+r1*cos(q1+theta0))+l2*((-1)*q1dot+(-1)*q2dot+(-1)* 
      theta0dot)*cos(q1+q2+theta0)+(-1)*theta0dot*(r0x*cos(theta0)+(-1)*r0y* 
      sin(theta0)))*((-1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)* 
      q1dot+(-1)*theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0))+l2*((-1)* 
      q1dot+(-1)*q2dot+(-1)*theta0dot)*sin(q1+q2+theta0))+2*((q1dot+ 
      theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+l2*(q1dot+q2dot+ 
      theta0dot)*cos(q1+q2+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0) 
      ))*(theta0dot*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0))+(q1dot+ 
      theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+(-1)*l2*( 
      q1dot+q2dot+theta0dot)*sin(q1+q2+theta0)))+(-1)*m3*(2*(((-1)*q1dot+ 
      (-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+((-1)*q1dot+(-1) 
      *q2dot+(-1)*theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+ 
      l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+ 
      q3+theta0)+(-1)*theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*((-1) 
      *theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot) 
      *(l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1) 
      *theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+q2+theta0))+l3*((-1)* 
      q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*sin(q1+q2+q3+theta0))+ 
      2*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+(q1dot+ 
      q2dot+theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*(q1dot+ 
      q2dot+q3dot+theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x*cos(theta0)+(-1) 
      *r0y*sin(theta0)))*(theta0dot*((-1)*r0y*cos(theta0)+(-1)*r0x*sin( 
      theta0))+(q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0) 
      )+(q1dot+q2dot+theta0dot)*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin( 
      q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+theta0dot)*sin(q1+q2+q3+ 
      theta0))))+q2dot*(xc0dot*((-1)*l2*m2*cos(q1+q2+theta0)+m3*((-1)* 
      l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)*l3*cos(q1+q2+ 
      q3+theta0)))+yc0dot*((-1)*l2*m2*sin(q1+q2+theta0)+m3*((-1)*l2*sin( 
      q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0)))+ 
      (1/2)*(m2*((-2)*l2*(q1dot+q2dot+theta0dot)*(r0x*cos(theta0)+l1* 
      cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+(-1)*r0y*sin(theta0) 
      )*sin(q1+q2+theta0)+(-2)*l2*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1* 
      cos(q1+theta0))+l2*(q1dot+q2dot+theta0dot)*cos(q1+q2+theta0)+theta0dot*( 
      r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*sin(q1+q2+theta0)+2*l2*((-1)* 
      q1dot+(-1)*q2dot+(-1)*theta0dot)*cos(q1+q2+theta0)*((-1)*r0y*cos( 
      theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+ 
      theta0)+(-1)*l2*sin(q1+q2+theta0))+(-2)*l2*cos(q1+q2+theta0)*((-1)* 
      theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)* 
      (l1*sin(q1+theta0)+r1*sin(q1+theta0))+l2*((-1)*q1dot+(-1)*q2dot+( 
      -1)*theta0dot)*sin(q1+q2+theta0)))+m3*(2*((q1dot+theta0dot)*(l1*cos( 
      q1+theta0)+r1*cos(q1+theta0))+(q1dot+q2dot+theta0dot)*(l2*cos(q1+q2+theta0) 
      +r2*cos(q1+q2+theta0))+l3*(q1dot+q2dot+q3dot+theta0dot)*cos(q1+q2+q3+ 
      theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*((-1)*l2*sin( 
      q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))+ 
      2*(((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*cos(q1+q2+theta0)+ 
      r2*cos(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1) 
      *theta0dot)*cos(q1+q2+q3+theta0))*((-1)*r0y*cos(theta0)+(-1)*r0x* 
      sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2* 
      sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+ 
      theta0))+2*((-1)*l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)* 
      l3*cos(q1+q2+q3+theta0))*((-1)*theta0dot*(r0y*cos(theta0)+r0x*sin( 
      theta0))+((-1)*q1dot+(-1)*theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0) 
      )+((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*sin(q1+q2+theta0)+r2* 
      sin(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)* 
      theta0dot)*sin(q1+q2+q3+theta0))+2*(r0x*cos(theta0)+l1*cos(q1+theta0)+r1* 
      cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+ 
      q3+theta0)+(-1)*r0y*sin(theta0))*((q1dot+q2dot+theta0dot)*((-1)*l2* 
      sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+ 
      q3dot+theta0dot)*sin(q1+q2+q3+theta0)))))+q1dot*(xc0dot*((-1)*l1* 
      m1*cos(q1+theta0)+m2*((-1)*l1*cos(q1+theta0)+(-1)*r1*cos(q1+theta0)+( 
      -1)*l2*cos(q1+q2+theta0))+m3*((-1)*l1*cos(q1+theta0)+(-1)*r1*cos( 
      q1+theta0)+(-1)*l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)* 
      l3*cos(q1+q2+q3+theta0)))+yc0dot*((-1)*l1*m1*sin(q1+theta0)+m2*(( 
      -1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+ 
      theta0))+m3*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2* 
      sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+ 
      theta0)))+(1/2)*(m1*((-2)*l1*(q1dot+theta0dot)*(r0x*cos(theta0)+l1* 
      cos(q1+theta0)+(-1)*r0y*sin(theta0))*sin(q1+theta0)+(-2)*l1*(l1*( 
      q1dot+theta0dot)*cos(q1+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin( 
      theta0)))*sin(q1+theta0)+2*l1*((-1)*q1dot+(-1)*theta0dot)*cos(q1+theta0) 
      *((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0))+ 
      (-2)*l1*cos(q1+theta0)*((-1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0) 
      )+l1*((-1)*q1dot+(-1)*theta0dot)*sin(q1+theta0)))+m2*(2*((q1dot+ 
      theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+l2*(q1dot+q2dot+ 
      theta0dot)*cos(q1+q2+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0) 
      ))*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+ 
      q2+theta0))+2*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos( 
      q1+theta0))+l2*((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*cos(q1+q2+ 
      theta0))*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+ 
      theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0))+2*((-1)* 
      l1*cos(q1+theta0)+(-1)*r1*cos(q1+theta0)+(-1)*l2*cos(q1+q2+theta0))*( 
      (-1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)* 
      theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0))+l2*((-1)*q1dot+(-1)* 
      q2dot+(-1)*theta0dot)*sin(q1+q2+theta0))+2*(r0x*cos(theta0)+l1*cos(q1+ 
      theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+(-1)*r0y*sin(theta0))*(( 
      q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+(-1) 
      *l2*(q1dot+q2dot+theta0dot)*sin(q1+q2+theta0)))+m3*(2*((q1dot+ 
      theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+(q1dot+q2dot+theta0dot)*( 
      l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*(q1dot+q2dot+q3dot+ 
      theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin( 
      theta0)))*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2* 
      sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+ 
      theta0))+2*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+ 
      theta0))+((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*cos(q1+q2+theta0)+ 
      r2*cos(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1) 
      *theta0dot)*cos(q1+q2+q3+theta0))*((-1)*r0y*cos(theta0)+(-1)*r0x* 
      sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2* 
      sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+ 
      theta0))+2*((-1)*l1*cos(q1+theta0)+(-1)*r1*cos(q1+theta0)+(-1)*l2* 
      cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)*l3*cos(q1+q2+q3+ 
      theta0))*((-1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+( 
      -1)*theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)*q1dot+(-1) 
      *q2dot+(-1)*theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+q2+theta0))+ 
      l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*sin(q1+q2+ 
      q3+theta0))+2*(r0x*cos(theta0)+l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2* 
      cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0)+(-1)* 
      r0y*sin(theta0))*((q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)*r1* 
      sin(q1+theta0))+(q1dot+q2dot+theta0dot)*((-1)*l2*sin(q1+q2+theta0)+(-1) 
      *r2*sin(q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+theta0dot)*sin( 
      q1+q2+q3+theta0)))))+theta0dot*(xc0dot*(m1*((-1)*r0x*cos(theta0)+(-1)* 
      l1*cos(q1+theta0)+r0y*sin(theta0))+m2*((-1)*r0x*cos(theta0)+(-1)*l1* 
      cos(q1+theta0)+(-1)*r1*cos(q1+theta0)+(-1)*l2*cos(q1+q2+theta0)+r0y* 
      sin(theta0))+m3*((-1)*r0x*cos(theta0)+(-1)*l1*cos(q1+theta0)+(-1)* 
      r1*cos(q1+theta0)+(-1)*l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+ 
      (-1)*l3*cos(q1+q2+q3+theta0)+r0y*sin(theta0)))+yc0dot*(m1*((-1)* 
      r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0))+m2*((-1) 
      *r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)* 
      r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0))+m3*((-1)*r0y*cos(theta0) 
      +(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+( 
      -1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+ 
      q2+q3+theta0)))+(1/2)*(m1*(2*(l1*((-1)*q1dot+(-1)*theta0dot)*cos( 
      q1+theta0)+(-1)*theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*((-1) 
      *r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0))+2*( 
      l1*(q1dot+theta0dot)*cos(q1+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y* 
      sin(theta0)))*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1* 
      sin(q1+theta0))+2*((-1)*r0x*cos(theta0)+(-1)*l1*cos(q1+theta0)+r0y* 
      sin(theta0))*((-1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+l1*((-1) 
      *q1dot+(-1)*theta0dot)*sin(q1+theta0))+2*(r0x*cos(theta0)+l1*cos(q1+ 
      theta0)+(-1)*r0y*sin(theta0))*(theta0dot*((-1)*r0y*cos(theta0)+(-1)* 
      r0x*sin(theta0))+(-1)*l1*(q1dot+theta0dot)*sin(q1+theta0)))+m2*(2*((( 
      -1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+l2*(( 
      -1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*cos(q1+q2+theta0)+(-1)* 
      theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*((-1)*r0y*cos(theta0) 
      +(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+( 
      -1)*l2*sin(q1+q2+theta0))+2*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1* 
      cos(q1+theta0))+l2*(q1dot+q2dot+theta0dot)*cos(q1+q2+theta0)+theta0dot*( 
      r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*((-1)*r0y*cos(theta0)+(-1)* 
      r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)* 
      l2*sin(q1+q2+theta0))+2*((-1)*r0x*cos(theta0)+(-1)*l1*cos(q1+theta0)+ 
      (-1)*r1*cos(q1+theta0)+(-1)*l2*cos(q1+q2+theta0)+r0y*sin(theta0))*(( 
      -1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)* 
      theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0))+l2*((-1)*q1dot+(-1)* 
      q2dot+(-1)*theta0dot)*sin(q1+q2+theta0))+2*(r0x*cos(theta0)+l1*cos(q1+ 
      theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+(-1)*r0y*sin(theta0))*( 
      theta0dot*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0))+(q1dot+theta0dot)* 
      ((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+(-1)*l2*(q1dot+ 
      q2dot+theta0dot)*sin(q1+q2+theta0)))+m3*(2*(((-1)*q1dot+(-1)* 
      theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+((-1)*q1dot+(-1)* 
      q2dot+(-1)*theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*( 
      (-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+ 
      theta0)+(-1)*theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*((-1)* 
      r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1* 
      sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1) 
      *l3*sin(q1+q2+q3+theta0))+2*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1* 
      cos(q1+theta0))+(q1dot+q2dot+theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+ 
      q2+theta0))+l3*(q1dot+q2dot+q3dot+theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))+2*((-1)*r0x*cos(theta0)+(-1)*l1*cos(q1+theta0)+(-1)*r1*cos(q1+theta0)+(-1)*l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)*l3*cos(q1+q2+q3+theta0)+r0y*sin(theta0))*((-1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*sin(q1+q2+q3+theta0))+2*(r0x*cos(theta0)+l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0)+(-1)*r0y*sin(theta0))*(theta0dot*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0))+(q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+(q1dot+q2dot+theta0dot)*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+theta0dot)*sin(q1+q2+q3+theta0)))));

      c41 = (-1)*xc0dot*(l1*m1*((-1)*q1dot+(-1)*theta0dot)*cos(q1+theta0)+ 
      m2*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0)) 
      +l2*((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*cos(q1+q2+theta0))+m3*( 
      ((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+(( 
      -1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos( 
      q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot) 
      *cos(q1+q2+q3+theta0)))+(-1)*yc0dot*((-1)*l1*m1*(q1dot+theta0dot)* 
      sin(q1+theta0)+m2*((q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)*r1* 
      sin(q1+theta0))+(-1)*l2*(q1dot+q2dot+theta0dot)*sin(q1+q2+theta0))+m3*( 
      (q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+( 
      q1dot+q2dot+theta0dot)*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+ 
      q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+theta0dot)*sin(q1+q2+q3+theta0))) 
      +q3dot*((-1)*l3*m3*xc0dot*cos(q1+q2+q3+theta0)+(-1)*l3*m3* 
      yc0dot*sin(q1+q2+q3+theta0)+(1/2)*m3*((-2)*l3*(q1dot+q2dot+q3dot+ 
      theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2* 
      cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0))*sin(q1+q2+q3+theta0)+(-2)* 
      l3*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+(q1dot+ 
      q2dot+theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*(q1dot+ 
      q2dot+q3dot+theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x*cos(theta0)+(-1) 
      *r0y*sin(theta0)))*sin(q1+q2+q3+theta0)+2*l3*((-1)*q1dot+(-1)* 
      q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0)*((-1)*l1* 
      sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)* 
      r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))+(-2)*l3*cos(q1+ 
      q2+q3+theta0)*((-1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)* 
      q1dot+(-1)*theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)* 
      q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+ 
      q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)* 
      sin(q1+q2+q3+theta0))))+(1/2)*((-1)*m1*((-2)*l1*(q1dot+theta0dot)* 
      (l1*(q1dot+theta0dot)*cos(q1+theta0)+theta0dot*(r0x*cos(theta0)+(-1)* 
      r0y*sin(theta0)))*sin(q1+theta0)+2*l1*((-1)*q1dot+(-1)*theta0dot)* 
      cos(q1+theta0)*((-1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+l1*(( 
      -1)*q1dot+(-1)*theta0dot)*sin(q1+theta0)))+(-1)*m2*(2*(((-1)* 
      q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+l2*((-1)* 
      q1dot+(-1)*q2dot+(-1)*theta0dot)*cos(q1+q2+theta0))*((-1)*theta0dot*( 
      r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)*(l1*sin( 
      q1+theta0)+r1*sin(q1+theta0))+l2*((-1)*q1dot+(-1)*q2dot+(-1)* 
      theta0dot)*sin(q1+q2+theta0))+2*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1* 
      cos(q1+theta0))+l2*(q1dot+q2dot+theta0dot)*cos(q1+q2+theta0)+theta0dot*( 
      r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*((q1dot+theta0dot)*((-1)*l1* 
      sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+(-1)*l2*(q1dot+q2dot+theta0dot) 
      *sin(q1+q2+theta0)))+(-1)*m3*(2*(((-1)*q1dot+(-1)*theta0dot)*( 
      l1*cos(q1+theta0)+r1*cos(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)* 
      theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*((-1)*q1dot+ 
      (-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0))*((-1)* 
      theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)* 
      (l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)* 
      theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+q2+theta0))+l3*((-1)*q1dot+ 
      (-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*sin(q1+q2+q3+theta0))+2*(( 
      q1dot+theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+(q1dot+q2dot+ 
      theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*(q1dot+q2dot+ 
      q3dot+theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x*cos(theta0)+(-1)* 
      r0y*sin(theta0)))*((q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)* 
      r1*sin(q1+theta0))+(q1dot+q2dot+theta0dot)*((-1)*l2*sin(q1+q2+theta0)+( 
      -1)*r2*sin(q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+theta0dot)* 
      sin(q1+q2+q3+theta0))))+q2dot*(xc0dot*((-1)*l2*m2*cos(q1+q2+theta0)+ 
      m3*((-1)*l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)*l3* 
      cos(q1+q2+q3+theta0)))+yc0dot*((-1)*l2*m2*sin(q1+q2+theta0)+m3*((-1) 
      *l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+ 
      q3+theta0)))+(1/2)*(m2*((-2)*l2*(q1dot+q2dot+theta0dot)*(l1*cos( 
      q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0))*sin(q1+q2+theta0)+(-2)* 
      l2*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+l2*(q1dot+ 
      q2dot+theta0dot)*cos(q1+q2+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y* 
      sin(theta0)))*sin(q1+q2+theta0)+2*l2*((-1)*q1dot+(-1)*q2dot+(-1)* 
      theta0dot)*cos(q1+q2+theta0)*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+ 
      theta0)+(-1)*l2*sin(q1+q2+theta0))+(-2)*l2*cos(q1+q2+theta0)*((-1)* 
      theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)* 
      (l1*sin(q1+theta0)+r1*sin(q1+theta0))+l2*((-1)*q1dot+(-1)*q2dot+( 
      -1)*theta0dot)*sin(q1+q2+theta0)))+m3*(2*((q1dot+theta0dot)*(l1*cos( 
      q1+theta0)+r1*cos(q1+theta0))+(q1dot+q2dot+theta0dot)*(l2*cos(q1+q2+theta0) 
      +r2*cos(q1+q2+theta0))+l3*(q1dot+q2dot+q3dot+theta0dot)*cos(q1+q2+q3+ 
      theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*((-1)*l2*sin( 
      q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))+ 
      2*(((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*cos(q1+q2+theta0)+ 
      r2*cos(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1) 
      *theta0dot)*cos(q1+q2+q3+theta0))*((-1)*l1*sin(q1+theta0)+(-1)*r1* 
      sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1) 
      *l3*sin(q1+q2+q3+theta0))+2*((-1)*l2*cos(q1+q2+theta0)+(-1)*r2* 
      cos(q1+q2+theta0)+(-1)*l3*cos(q1+q2+q3+theta0))*((-1)*theta0dot*(r0y* 
      cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)*(l1*sin(q1+ 
      theta0)+r1*sin(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*( 
      l2*sin(q1+q2+theta0)+r2*sin(q1+q2+theta0))+l3*((-1)*q1dot+(-1)* 
      q2dot+(-1)*q3dot+(-1)*theta0dot)*sin(q1+q2+q3+theta0))+2*(l1*cos( 
      q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3* 
      cos(q1+q2+q3+theta0))*((q1dot+q2dot+theta0dot)*((-1)*l2*sin(q1+q2+ 
      theta0)+(-1)*r2*sin(q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+ 
      theta0dot)*sin(q1+q2+q3+theta0)))))+q1dot*(xc0dot*((-1)*l1*m1*cos( 
      q1+theta0)+m2*((-1)*l1*cos(q1+theta0)+(-1)*r1*cos(q1+theta0)+(-1)* 
      l2*cos(q1+q2+theta0))+m3*((-1)*l1*cos(q1+theta0)+(-1)*r1*cos(q1+ 
      theta0)+(-1)*l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)*l3* 
      cos(q1+q2+q3+theta0)))+yc0dot*((-1)*l1*m1*sin(q1+theta0)+m2*((-1)* 
      l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0))+ 
      m3*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+ 
      q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0)))+( 
      1/2)*(m1*((-2)*pow(l1,2)*((-1)*q1dot+(-1)*theta0dot)*cos(q1+theta0)* 
      sin(q1+theta0)+(-2)*pow(l1,2)*(q1dot+theta0dot)*cos(q1+theta0)*sin(q1+theta0)+ 
      (-2)*l1*(l1*(q1dot+theta0dot)*cos(q1+theta0)+theta0dot*(r0x*cos(theta0)+ 
      (-1)*r0y*sin(theta0)))*sin(q1+theta0)+(-2)*l1*cos(q1+theta0)*((-1)* 
      theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+l1*((-1)*q1dot+(-1)* 
      theta0dot)*sin(q1+theta0)))+m2*(2*(((-1)*q1dot+(-1)*theta0dot)*(l1* 
      cos(q1+theta0)+r1*cos(q1+theta0))+l2*((-1)*q1dot+(-1)*q2dot+(-1)* 
      theta0dot)*cos(q1+q2+theta0))*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+ 
      theta0)+(-1)*l2*sin(q1+q2+theta0))+2*((q1dot+theta0dot)*(l1*cos(q1+ 
      theta0)+r1*cos(q1+theta0))+l2*(q1dot+q2dot+theta0dot)*cos(q1+q2+theta0)+ 
      theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*((-1)*l1*sin(q1+ 
      theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0))+2*((-1)* 
      l1*cos(q1+theta0)+(-1)*r1*cos(q1+theta0)+(-1)*l2*cos(q1+q2+theta0))*( 
      (-1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)* 
      theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0))+l2*((-1)*q1dot+(-1)* 
      q2dot+(-1)*theta0dot)*sin(q1+q2+theta0))+2*(l1*cos(q1+theta0)+r1*cos( 
      q1+theta0)+l2*cos(q1+q2+theta0))*((q1dot+theta0dot)*((-1)*l1*sin(q1+ 
      theta0)+(-1)*r1*sin(q1+theta0))+(-1)*l2*(q1dot+q2dot+theta0dot)*sin( 
      q1+q2+theta0)))+m3*(2*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0) 
      +r1*cos(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2* 
      cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+( 
      -1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0))*((-1)*l1*sin(q1+ 
      theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin( 
      q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))+2*((q1dot+theta0dot)*(l1* 
      cos(q1+theta0)+r1*cos(q1+theta0))+(q1dot+q2dot+theta0dot)*(l2*cos(q1+q2+ 
      theta0)+r2*cos(q1+q2+theta0))+l3*(q1dot+q2dot+q3dot+theta0dot)*cos(q1+ 
      q2+q3+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*((-1)* 
      l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+( 
      -1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))+2*((-1)* 
      l1*cos(q1+theta0)+(-1)*r1*cos(q1+theta0)+(-1)*l2*cos(q1+q2+theta0)+( 
      -1)*r2*cos(q1+q2+theta0)+(-1)*l3*cos(q1+q2+q3+theta0))*((-1)* 
      theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)* 
      (l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)* 
      theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+q2+theta0))+l3*((-1)*q1dot+ 
      (-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*sin(q1+q2+q3+theta0))+2*(l1* 
      cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+ 
      l3*cos(q1+q2+q3+theta0))*((q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+( 
      -1)*r1*sin(q1+theta0))+(q1dot+q2dot+theta0dot)*((-1)*l2*sin(q1+q2+ 
      theta0)+(-1)*r2*sin(q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+ 
      theta0dot)*sin(q1+q2+q3+theta0)))))+theta0dot*(xc0dot*((-1)*l1*m1*cos( 
      q1+theta0)+m2*((-1)*l1*cos(q1+theta0)+(-1)*r1*cos(q1+theta0)+(-1)* 
      l2*cos(q1+q2+theta0))+m3*((-1)*l1*cos(q1+theta0)+(-1)*r1*cos(q1+ 
      theta0)+(-1)*l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)*l3* 
      cos(q1+q2+q3+theta0)))+yc0dot*((-1)*l1*m1*sin(q1+theta0)+m2*((-1)* 
      l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0))+ 
      m3*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+ 
      q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0)))+( 
      1/2)*(m1*((-2)*l1*(l1*((-1)*q1dot+(-1)*theta0dot)*cos(q1+theta0) 
      +(-1)*theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*sin(q1+theta0)+( 
      -2)*l1*(l1*(q1dot+theta0dot)*cos(q1+theta0)+theta0dot*(r0x*cos(theta0)+( 
      -1)*r0y*sin(theta0)))*sin(q1+theta0)+(-2)*l1*cos(q1+theta0)*((-1)* 
      theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+l1*((-1)*q1dot+(-1)* 
      theta0dot)*sin(q1+theta0))+2*l1*cos(q1+theta0)*(theta0dot*((-1)*r0y* 
      cos(theta0)+(-1)*r0x*sin(theta0))+(-1)*l1*(q1dot+theta0dot)*sin(q1+ 
      theta0)))+m2*(2*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1* 
      cos(q1+theta0))+l2*((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*cos(q1+ 
      q2+theta0)+(-1)*theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*((-1) 
      *l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0))+ 
      2*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+l2*(q1dot+ 
      q2dot+theta0dot)*cos(q1+q2+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y* 
      sin(theta0)))*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)* 
      l2*sin(q1+q2+theta0))+2*((-1)*l1*cos(q1+theta0)+(-1)*r1*cos(q1+ 
      theta0)+(-1)*l2*cos(q1+q2+theta0))*((-1)*theta0dot*(r0y*cos(theta0)+ 
      r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)*(l1*sin(q1+theta0)+r1* 
      sin(q1+theta0))+l2*((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*sin(q1+ 
      q2+theta0))+2*(l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0))* 
      (theta0dot*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0))+(q1dot+theta0dot) 
      *((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+(-1)*l2*(q1dot+ 
      q2dot+theta0dot)*sin(q1+q2+theta0)))+m3*(2*(((-1)*q1dot+(-1)* 
      theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+((-1)*q1dot+(-1)* 
      q2dot+(-1)*theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*( 
      (-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+ 
      theta0)+(-1)*theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*((-1)* 
      l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+( 
      -1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))+2*((q1dot+ 
      theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+(q1dot+q2dot+theta0dot)*( 
      l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*(q1dot+q2dot+q3dot+ 
      theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin( 
      theta0)))*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2* 
      sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+ 
      theta0))+2*((-1)*l1*cos(q1+theta0)+(-1)*r1*cos(q1+theta0)+(-1)*l2* 
      cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)*l3*cos(q1+q2+q3+ 
      theta0))*((-1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+( 
      -1)*theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)*q1dot+(-1) 
      *q2dot+(-1)*theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+q2+theta0))+ 
      l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*sin(q1+q2+ 
      q3+theta0))+2*(l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+ 
      r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0))*(theta0dot*((-1)*r0y* 
      cos(theta0)+(-1)*r0x*sin(theta0))+(q1dot+theta0dot)*((-1)*l1*sin(q1+ 
      theta0)+(-1)*r1*sin(q1+theta0))+(q1dot+q2dot+theta0dot)*((-1)*l2*sin( 
      q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+ 
      theta0dot)*sin(q1+q2+q3+theta0)))));
        
        c51 = (-1)*xc0dot*(l2*m2*((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)* 
      cos(q1+q2+theta0)+m3*(((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2* 
      cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+( 
      -1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0)))+(-1)*yc0dot*((-1)* 
      l2*m2*(q1dot+q2dot+theta0dot)*sin(q1+q2+theta0)+m3*((q1dot+q2dot+ 
      theta0dot)*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0))+(-1) 
      *l3*(q1dot+q2dot+q3dot+theta0dot)*sin(q1+q2+q3+theta0)))+q3dot*((-1) 
      *l3*m3*xc0dot*cos(q1+q2+q3+theta0)+(-1)*l3*m3*yc0dot*sin(q1+q2+ 
      q3+theta0)+(1/2)*m3*((-2)*l3*(q1dot+q2dot+q3dot+theta0dot)*(l2* 
      cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0))*sin(q1+ 
      q2+q3+theta0)+(-2)*l3*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+ 
      theta0))+(q1dot+q2dot+theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0) 
      )+l3*(q1dot+q2dot+q3dot+theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x* 
      cos(theta0)+(-1)*r0y*sin(theta0)))*sin(q1+q2+q3+theta0)+2*l3*((-1)* 
      q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0)*(( 
      -1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+ 
      q2+q3+theta0))+(-2)*l3*cos(q1+q2+q3+theta0)*((-1)*theta0dot*(r0y*cos( 
      theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)*(l1*sin(q1+theta0)+ 
      r1*sin(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*sin( 
      q1+q2+theta0)+r2*sin(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)* 
      q3dot+(-1)*theta0dot)*sin(q1+q2+q3+theta0))))+(1/2)*((-1)*m2*((-2) 
      *l2*(q1dot+q2dot+theta0dot)*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1* 
      cos(q1+theta0))+l2*(q1dot+q2dot+theta0dot)*cos(q1+q2+theta0)+theta0dot*( 
      r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*sin(q1+q2+theta0)+2*l2*((-1)* 
      q1dot+(-1)*q2dot+(-1)*theta0dot)*cos(q1+q2+theta0)*((-1)*theta0dot*( 
      r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)*(l1*sin( 
      q1+theta0)+r1*sin(q1+theta0))+l2*((-1)*q1dot+(-1)*q2dot+(-1)* 
      theta0dot)*sin(q1+q2+theta0)))+(-1)*m3*(2*(((-1)*q1dot+(-1)*q2dot+ 
      (-1)*theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*((-1)* 
      q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0))*( 
      (-1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)* 
      theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)*q1dot+(-1)* 
      q2dot+(-1)*theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+q2+theta0))+l3*( 
      (-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*sin(q1+q2+q3+ 
      theta0))+2*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+( 
      q1dot+q2dot+theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*( 
      q1dot+q2dot+q3dot+theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x*cos( 
      theta0)+(-1)*r0y*sin(theta0)))*((q1dot+q2dot+theta0dot)*((-1)*l2*sin( 
      q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+ 
      theta0dot)*sin(q1+q2+q3+theta0))))+q2dot*(xc0dot*((-1)*l2*m2*cos( 
      q1+q2+theta0)+m3*((-1)*l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+ 
      (-1)*l3*cos(q1+q2+q3+theta0)))+yc0dot*((-1)*l2*m2*sin(q1+q2+theta0) 
      +m3*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3* 
      sin(q1+q2+q3+theta0)))+(1/2)*(m2*((-2)*pow(l2,2)*((-1)*q1dot+(-1)* 
      q2dot+(-1)*theta0dot)*cos(q1+q2+theta0)*sin(q1+q2+theta0)+(-2)*pow(l2,2)*( 
      q1dot+q2dot+theta0dot)*cos(q1+q2+theta0)*sin(q1+q2+theta0)+(-2)*l2*(( 
      q1dot+theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+l2*(q1dot+q2dot+ 
      theta0dot)*cos(q1+q2+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0) 
      ))*sin(q1+q2+theta0)+(-2)*l2*cos(q1+q2+theta0)*((-1)*theta0dot*(r0y* 
      cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)*(l1*sin(q1+ 
      theta0)+r1*sin(q1+theta0))+l2*((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)* 
      sin(q1+q2+theta0)))+m3*(2*(((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)* 
      (l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*((-1)*q1dot+(-1)* 
      q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0))*((-1)*l2* 
      sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+ 
      theta0))+2*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+( 
      q1dot+q2dot+theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*( 
      q1dot+q2dot+q3dot+theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x*cos( 
      theta0)+(-1)*r0y*sin(theta0)))*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2* 
      sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))+2*((-1)*l2*cos(q1+ 
      q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)*l3*cos(q1+q2+q3+theta0))*(( 
      -1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)* 
      theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)*q1dot+(-1)* 
      q2dot+(-1)*theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+q2+theta0))+l3*( 
      (-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*sin(q1+q2+q3+ 
      theta0))+2*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+ 
      theta0))*((q1dot+q2dot+theta0dot)*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2* 
      sin(q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+theta0dot)*sin(q1+q2+ 
      q3+theta0)))))+q1dot*(xc0dot*((-1)*l2*m2*cos(q1+q2+theta0)+m3*((-1) 
      *l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+theta0)+(-1)*l3*cos(q1+q2+ 
      q3+theta0)))+yc0dot*((-1)*l2*m2*sin(q1+q2+theta0)+m3*((-1)*l2*sin( 
      q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0)))+ 
      (1/2)*(m2*((-2)*l2*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+ 
      theta0)+r1*cos(q1+theta0))+l2*((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)* 
      cos(q1+q2+theta0))*sin(q1+q2+theta0)+(-2)*l2*((q1dot+theta0dot)*(l1* 
      cos(q1+theta0)+r1*cos(q1+theta0))+l2*(q1dot+q2dot+theta0dot)*cos(q1+q2+ 
      theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*sin(q1+q2+theta0)+ 
      (-2)*l2*cos(q1+q2+theta0)*((-1)*theta0dot*(r0y*cos(theta0)+r0x*sin( 
      theta0))+((-1)*q1dot+(-1)*theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0) 
      )+l2*((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*sin(q1+q2+theta0))+2* 
      l2*cos(q1+q2+theta0)*((q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)* 
      r1*sin(q1+theta0))+(-1)*l2*(q1dot+q2dot+theta0dot)*sin(q1+q2+theta0)))+ 
      m3*(2*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+ 
      theta0))+((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*cos(q1+q2+theta0)+ 
      r2*cos(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1) 
      *theta0dot)*cos(q1+q2+q3+theta0))*((-1)*l2*sin(q1+q2+theta0)+(-1)* 
      r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))+2*((q1dot+theta0dot) 
      *(l1*cos(q1+theta0)+r1*cos(q1+theta0))+(q1dot+q2dot+theta0dot)*(l2* 
      cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*(q1dot+q2dot+q3dot+theta0dot) 
      *cos(q1+q2+q3+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))* 
      ((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin( 
      q1+q2+q3+theta0))+2*((-1)*l2*cos(q1+q2+theta0)+(-1)*r2*cos(q1+q2+ 
      theta0)+(-1)*l3*cos(q1+q2+q3+theta0))*((-1)*theta0dot*(r0y*cos(theta0)+ 
      r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)*(l1*sin(q1+theta0)+r1* 
      sin(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*sin(q1+ 
      q2+theta0)+r2*sin(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)* 
      q3dot+(-1)*theta0dot)*sin(q1+q2+q3+theta0))+2*(l2*cos(q1+q2+theta0)+ 
      r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0))*((q1dot+theta0dot)*((-1) 
      *l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+(q1dot+q2dot+theta0dot)*(( 
      -1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0))+(-1)*l3*( 
      q1dot+q2dot+q3dot+theta0dot)*sin(q1+q2+q3+theta0)))))+theta0dot*(xc0dot*( 
      (-1)*l2*m2*cos(q1+q2+theta0)+m3*((-1)*l2*cos(q1+q2+theta0)+(-1)* 
      r2*cos(q1+q2+theta0)+(-1)*l3*cos(q1+q2+q3+theta0)))+yc0dot*((-1)* 
      l2*m2*sin(q1+q2+theta0)+m3*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2* 
      sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0)))+(1/2)*(m2*((-2)* 
      l2*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0)) 
      +l2*((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*cos(q1+q2+theta0)+(-1)* 
      theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*sin(q1+q2+theta0)+(-2) 
      *l2*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+l2*( 
      q1dot+q2dot+theta0dot)*cos(q1+q2+theta0)+theta0dot*(r0x*cos(theta0)+(-1)* 
      r0y*sin(theta0)))*sin(q1+q2+theta0)+(-2)*l2*cos(q1+q2+theta0)*((-1)* 
      theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)* 
      (l1*sin(q1+theta0)+r1*sin(q1+theta0))+l2*((-1)*q1dot+(-1)*q2dot+( 
      -1)*theta0dot)*sin(q1+q2+theta0))+2*l2*cos(q1+q2+theta0)*(theta0dot*(( 
      -1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0))+(q1dot+theta0dot)*((-1)* 
      l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+(-1)*l2*(q1dot+q2dot+ 
      theta0dot)*sin(q1+q2+theta0)))+m3*(2*(((-1)*q1dot+(-1)*theta0dot)*( 
      l1*cos(q1+theta0)+r1*cos(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)* 
      theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*((-1)*q1dot+ 
      (-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0)+(-1)* 
      theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin(theta0)))*((-1)*l2*sin(q1+ 
      q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))+2*( 
      (q1dot+theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+(q1dot+q2dot+ 
      theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*(q1dot+q2dot+ 
      q3dot+theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x*cos(theta0)+(-1)* 
      r0y*sin(theta0)))*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+ 
      theta0)+(-1)*l3*sin(q1+q2+q3+theta0))+2*((-1)*l2*cos(q1+q2+theta0)+( 
      -1)*r2*cos(q1+q2+theta0)+(-1)*l3*cos(q1+q2+q3+theta0))*((-1)* 
      theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)* 
      (l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)* 
      theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+q2+theta0))+l3*((-1)*q1dot+ 
      (-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*sin(q1+q2+q3+theta0))+2*(l2* 
      cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0))*( 
      theta0dot*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0))+(q1dot+theta0dot)* 
      ((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0))+(q1dot+q2dot+theta0dot) 
      *((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0))+(-1)*l3*( 
      q1dot+q2dot+q3dot+theta0dot)*sin(q1+q2+q3+theta0)))));

        c61 =  (-1)*l3*m3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot) 
      *xc0dot*cos(q1+q2+q3+theta0)+l3*m3*(q1dot+q2dot+q3dot+theta0dot)* 
      yc0dot*sin(q1+q2+q3+theta0)+(-1/2)*m3*((-2)*l3*(q1dot+q2dot+ 
      q3dot+theta0dot)*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+ 
      (q1dot+q2dot+theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3* 
      (q1dot+q2dot+q3dot+theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x*cos( 
      theta0)+(-1)*r0y*sin(theta0)))*sin(q1+q2+q3+theta0)+2*l3*((-1)*q1dot+ 
      (-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0)*((-1)* 
      theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)* 
      (l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)* 
      theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+q2+theta0))+l3*((-1)*q1dot+ 
      (-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*sin(q1+q2+q3+theta0)))+q3dot* 
      ((-1)*l3*m3*xc0dot*cos(q1+q2+q3+theta0)+(-1)*l3*m3*yc0dot*sin( 
      q1+q2+q3+theta0)+(1/2)*m3*((-2)*pow(l3,2)*((-1)*q1dot+(-1)*q2dot+( 
      -1)*q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0)*sin(q1+q2+q3+theta0)+( 
      -2)*pow(l3,2)*(q1dot+q2dot+q3dot+theta0dot)*cos(q1+q2+q3+theta0)*sin(q1+ 
      q2+q3+theta0)+(-2)*l3*((q1dot+theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+ 
      theta0))+(q1dot+q2dot+theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0) 
      )+l3*(q1dot+q2dot+q3dot+theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x* 
      cos(theta0)+(-1)*r0y*sin(theta0)))*sin(q1+q2+q3+theta0)+(-2)*l3*cos( 
      q1+q2+q3+theta0)*((-1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1) 
      *q1dot+(-1)*theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)* 
      q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+ 
      q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)* 
      sin(q1+q2+q3+theta0))))+q2dot*((-1)*l3*m3*xc0dot*cos(q1+q2+q3+ 
      theta0)+(-1)*l3*m3*yc0dot*sin(q1+q2+q3+theta0)+(1/2)*m3*((-2)*l3* 
      (((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*cos(q1+q2+theta0)+r2* 
      cos(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)* 
      theta0dot)*cos(q1+q2+q3+theta0))*sin(q1+q2+q3+theta0)+(-2)*l3*((q1dot+ 
      theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+(q1dot+q2dot+theta0dot)*( 
      l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*(q1dot+q2dot+q3dot+ 
      theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin( 
      theta0)))*sin(q1+q2+q3+theta0)+(-2)*l3*cos(q1+q2+q3+theta0)*((-1)* 
      theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)* 
      (l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)* 
      theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+q2+theta0))+l3*((-1)*q1dot+ 
      (-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*sin(q1+q2+q3+theta0))+2*l3* 
      cos(q1+q2+q3+theta0)*((q1dot+q2dot+theta0dot)*((-1)*l2*sin(q1+q2+ 
      theta0)+(-1)*r2*sin(q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+ 
      theta0dot)*sin(q1+q2+q3+theta0))))+q1dot*((-1)*l3*m3*xc0dot*cos(q1+ 
      q2+q3+theta0)+(-1)*l3*m3*yc0dot*sin(q1+q2+q3+theta0)+(1/2)*m3*((-2) 
      *l3*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+ 
      theta0))+((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*cos(q1+q2+theta0)+ 
      r2*cos(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1) 
      *theta0dot)*cos(q1+q2+q3+theta0))*sin(q1+q2+q3+theta0)+(-2)*l3*(( 
      q1dot+theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+(q1dot+q2dot+ 
      theta0dot)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*(q1dot+q2dot+ 
      q3dot+theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x*cos(theta0)+(-1)* 
      r0y*sin(theta0)))*sin(q1+q2+q3+theta0)+(-2)*l3*cos(q1+q2+q3+theta0)*(( 
      -1)*theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)* 
      theta0dot)*(l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)*q1dot+(-1)* 
      q2dot+(-1)*theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+q2+theta0))+l3*( 
      (-1)*q1dot+(-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*sin(q1+q2+q3+ 
      theta0))+2*l3*cos(q1+q2+q3+theta0)*((q1dot+theta0dot)*((-1)*l1*sin( 
      q1+theta0)+(-1)*r1*sin(q1+theta0))+(q1dot+q2dot+theta0dot)*((-1)*l2* 
      sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+ 
      q3dot+theta0dot)*sin(q1+q2+q3+theta0))))+theta0dot*((-1)*l3*m3*xc0dot* 
      cos(q1+q2+q3+theta0)+(-1)*l3*m3*yc0dot*sin(q1+q2+q3+theta0)+(1/2)* 
      m3*((-2)*l3*(((-1)*q1dot+(-1)*theta0dot)*(l1*cos(q1+theta0)+r1* 
      cos(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)*theta0dot)*(l2*cos(q1+ 
      q2+theta0)+r2*cos(q1+q2+theta0))+l3*((-1)*q1dot+(-1)*q2dot+(-1)* 
      q3dot+(-1)*theta0dot)*cos(q1+q2+q3+theta0)+(-1)*theta0dot*(r0x*cos( 
      theta0)+(-1)*r0y*sin(theta0)))*sin(q1+q2+q3+theta0)+(-2)*l3*((q1dot+ 
      theta0dot)*(l1*cos(q1+theta0)+r1*cos(q1+theta0))+(q1dot+q2dot+theta0dot)*( 
      l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0))+l3*(q1dot+q2dot+q3dot+ 
      theta0dot)*cos(q1+q2+q3+theta0)+theta0dot*(r0x*cos(theta0)+(-1)*r0y*sin( 
      theta0)))*sin(q1+q2+q3+theta0)+(-2)*l3*cos(q1+q2+q3+theta0)*((-1)* 
      theta0dot*(r0y*cos(theta0)+r0x*sin(theta0))+((-1)*q1dot+(-1)*theta0dot)* 
      (l1*sin(q1+theta0)+r1*sin(q1+theta0))+((-1)*q1dot+(-1)*q2dot+(-1)* 
      theta0dot)*(l2*sin(q1+q2+theta0)+r2*sin(q1+q2+theta0))+l3*((-1)*q1dot+ 
      (-1)*q2dot+(-1)*q3dot+(-1)*theta0dot)*sin(q1+q2+q3+theta0))+2*l3* 
      cos(q1+q2+q3+theta0)*(theta0dot*((-1)*r0y*cos(theta0)+(-1)*r0x*sin( 
      theta0))+(q1dot+theta0dot)*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0) 
      )+(q1dot+q2dot+theta0dot)*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin( 
      q1+q2+theta0))+(-1)*l3*(q1dot+q2dot+q3dot+theta0dot)*sin(q1+q2+q3+ 
      theta0))));
    
    c(0) = c11;
    c(1) = c21;
    c(2) = c31;
    c(3) = c41;
    c(4) = c51;
    c(5) = c61;

    h11 = m1 + m2 + m3 + m0;
    h12 = 0;
    
    h13 = m1*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin( 
    q1+theta0))+m2*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1* 
    sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0))+m3*(( 
    -1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1) 
    *r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+ 
    theta0)+(-1)*l3*sin(q1+q2+q3+theta0));
  
    h14 = (-1)*l1*m1*sin(q1+theta0)+m2*(( 
    -1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+ 
    theta0))+m3*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2* 
    sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+ 
    theta0)) ;
    
    h15 = (-1)*l2*m2*sin(q1+q2+theta0)+m3*((-1)*l2*sin(q1+q2+theta0)+( 
    -1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0));
      h16 =(-1)*l3* 
    m3*sin(q1+q2+q3+theta0);
      ///////////////
    h21 = 0;

    h22 = m1 + m2 + m3 + m0;

    h23 = m1*(r0x*cos(theta0)+l1*cos(q1+theta0)+(-1) 
    *r0y*sin(theta0))+m2*(r0x*cos(theta0)+l1*cos(q1+theta0)+r1*cos(q1+ 
    theta0)+l2*cos(q1+q2+theta0)+(-1)*r0y*sin(theta0))+m3*(r0x*cos(theta0)+ 
    l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+q2+ 
    theta0)+l3*cos(q1+q2+q3+theta0)+(-1)*r0y*sin(theta0));

    h24 = l1*m1*cos(q1+ 
    theta0)+m2*(l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0))+m3* 
    (l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+q2+ 
    theta0)+l3*cos(q1+q2+q3+theta0));

    h25 = l2*m2*cos(q1+q2+theta0)+m3*(l2*cos( 
    q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0));

    h26 = l3*m3*cos( 
    q1+q2+q3+theta0);
      ///////////////
    h31 = m1*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)* 
    l1*sin(q1+theta0))+m2*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1) 
    *l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0))+ 
    m3*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0) 
    +(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+ 
    q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0));

    h32 = m1*(r0x*cos(theta0)+l1*cos( 
    q1+theta0)+(-1)*r0y*sin(theta0))+m2*(r0x*cos(theta0)+l1*cos(q1+theta0)+ 
    r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+(-1)*r0y*sin(theta0))+m3*(r0x* 
    cos(theta0)+l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2* 
    cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0)+(-1)*r0y*sin(theta0)) ;

    h33 = (1/2)*( 
    2*ibzz+2*i1zz+2*i2zz+2*i3zz+m1*(2*pow((r0x*cos(theta0)+l1*cos(q1+theta0) 
    +(-1)*r0y*sin(theta0)),2)+2*pow(((-1)*r0y*cos(theta0)+(-1)*r0x*sin( 
    theta0)+(-1)*l1*sin(q1+theta0)),2))+m2*(2*pow((r0x*cos(theta0)+l1*cos(q1+ 
    theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+(-1)*r0y*sin(theta0)),2)+ 
    2*pow(((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+ 
    (-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)),2))+m3*(2*pow((r0x* 
    cos(theta0)+l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2* 
    cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0)+(-1)*r0y*sin(theta0)),2)+2*pow((( 
    -1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1) 
    *r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+ 
    theta0)+(-1)*l3*sin(q1+q2+q3+theta0)),2)));
    
    h34 = (1/2)*(2*i1zz+2*i2zz+2* 
    i3zz+m1*(2*l1*cos(q1+theta0)*(r0x*cos(theta0)+l1*cos(q1+theta0)+(-1)* 
    r0y*sin(theta0))+(-2)*l1*sin(q1+theta0)*((-1)*r0y*cos(theta0)+(-1)* 
    r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)))+m2*(2*(l1*cos(q1+theta0)+ 
    r1*cos(q1+theta0)+l2*cos(q1+q2+theta0))*(r0x*cos(theta0)+l1*cos(q1+ 
    theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+(-1)*r0y*sin(theta0))+2*(( 
    -1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+ 
    theta0))*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+ 
    theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)))+m3*(2*( 
    l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+q2+ 
    theta0)+l3*cos(q1+q2+q3+theta0))*(r0x*cos(theta0)+l1*cos(q1+theta0)+r1* 
    cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+ 
    q3+theta0)+(-1)*r0y*sin(theta0))+2*((-1)*l1*sin(q1+theta0)+(-1)*r1* 
    sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1) 
    *l3*sin(q1+q2+q3+theta0))*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0) 
    +(-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+ 
    theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))));
    
    h35 = (1/2) 
    *(2*i2zz+2*i3zz+m2*(2*l2*cos(q1+q2+theta0)*(r0x*cos(theta0)+l1* 
    cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+(-1)*r0y*sin(theta0) 
    )+(-2)*l2*sin(q1+q2+theta0)*((-1)*r0y*cos(theta0)+(-1)*r0x*sin( 
    theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+ 
    q2+theta0)))+m3*(2*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos( 
    q1+q2+q3+theta0))*(r0x*cos(theta0)+l1*cos(q1+theta0)+r1*cos(q1+theta0)+ 
    l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0)+(-1)* 
    r0y*sin(theta0))+2*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+ 
    theta0)+(-1)*l3*sin(q1+q2+q3+theta0))*((-1)*r0y*cos(theta0)+(-1)* 
    r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)* 
    l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+ 
    q3+theta0))));
    
    h36 =  (1/2)*(2*i3zz+m3*(2*l3*cos(q1+q2+q3+theta0)*(r0x* 
    cos(theta0)+l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2* 
    cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0)+(-1)*r0y*sin(theta0))+(-2)* 
    l3*sin(q1+q2+q3+theta0)*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+( 
    -1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+ 
    theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))));
      //////////////
    
    h41 = (-1) 
    *l1*m1*sin(q1+theta0)+m2*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+ 
    theta0)+(-1)*l2*sin(q1+q2+theta0))+m3*((-1)*l1*sin(q1+theta0)+(-1)* 
    r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+ 
    (-1)*l3*sin(q1+q2+q3+theta0));
    
    h42 = l1*m1*cos(q1+theta0)+m2*(l1*cos(q1+ 
    theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0))+m3*(l1*cos(q1+theta0)+r1* 
    cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+ 
    q3+theta0));
    
    h43 = (1/2)*(2*i1zz+2*i2zz+2*i3zz+m1*(2*l1*cos(q1+theta0)*( 
    r0x*cos(theta0)+l1*cos(q1+theta0)+(-1)*r0y*sin(theta0))+(-2)*l1*sin( 
    q1+theta0)*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin( 
    q1+theta0)))+m2*(2*(l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+ 
    theta0))*(r0x*cos(theta0)+l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+ 
    q2+theta0)+(-1)*r0y*sin(theta0))+2*((-1)*l1*sin(q1+theta0)+(-1)*r1* 
    sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0))*((-1)*r0y*cos(theta0)+(-1)* 
    r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)* 
    l2*sin(q1+q2+theta0)))+m3*(2*(l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2* 
    cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0))*(r0x* 
    cos(theta0)+l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2* 
    cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0)+(-1)*r0y*sin(theta0))+2*((-1) 
    *l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+( 
    -1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))*((-1)*r0y* 
    cos(theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1*sin( 
    q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)* 
    l3*sin(q1+q2+q3+theta0))));
    
    h44 = (1/2)*(2*i1zz+2*i2zz+2*i3zz+m1*(2* 
    pow(l1,2)*pow(cos(q1+theta0),2)+2*pow(l1,2)*pow(sin(q1+theta0),2))+m2*(2*pow((l1*cos( 
    q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)),2)+2*pow(((-1)*l1*sin( 
    q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)),2))+m3*( 
    2*pow((l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+ 
    q2+theta0)+l3*cos(q1+q2+q3+theta0)),2)+2*pow(((-1)*l1*sin(q1+theta0)+(-1)* 
    r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+ 
    (-1)*l3*sin(q1+q2+q3+theta0)),2)));
    
    h45 = (1/2)*(2*i2zz+2*i3zz+m2*(2* 
    l2*cos(q1+q2+theta0)*(l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+ 
    q2+theta0))+(-2)*l2*sin(q1+q2+theta0)*((-1)*l1*sin(q1+theta0)+(-1)* 
    r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)))+m3*(2*(l2*cos(q1+q2+ 
    theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0))*(l1*cos(q1+theta0)+ 
    r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+ 
    q2+q3+theta0))+2*((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+ 
    (-1)*l3*sin(q1+q2+q3+theta0))*((-1)*l1*sin(q1+theta0)+(-1)*r1* 
    sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1) 
    *l3*sin(q1+q2+q3+theta0))));
    
    h46 = (1/2)*(2*i3zz+m3*(2*l3*cos(q1+q2+ 
    q3+theta0)*(l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2* 
    cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0))+(-2)*l3*sin(q1+q2+q3+theta0) 
    *((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+ 
    q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))));
      //////////////
    h51 =( 
    -1)*l2*m2*sin(q1+q2+theta0)+m3*((-1)*l2*sin(q1+q2+theta0)+(-1)* 
    r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0));
    
    h52 = l2*m2*cos(q1+q2+ 
    theta0)+m3*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+ 
    theta0));
    
    h53 = (1/2)*(2*i2zz+2*i3zz+m2*(2*l2*cos(q1+q2+theta0)*(r0x*cos( 
    theta0)+l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+(-1)* 
    r0y*sin(theta0))+(-2)*l2*sin(q1+q2+theta0)*((-1)*r0y*cos(theta0)+(-1) 
    *r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)* 
    l2*sin(q1+q2+theta0)))+m3*(2*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+ 
    theta0)+l3*cos(q1+q2+q3+theta0))*(r0x*cos(theta0)+l1*cos(q1+theta0)+r1* 
    cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+ 
    q3+theta0)+(-1)*r0y*sin(theta0))+2*((-1)*l2*sin(q1+q2+theta0)+(-1)* 
    r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))*((-1)*r0y*cos( 
    theta0)+(-1)*r0x*sin(theta0)+(-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+ 
    theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3* 
    sin(q1+q2+q3+theta0))));
    
    h54 = (1/2)*(2*i2zz+2*i3zz+m2*(2*l2*cos(q1+q2+ 
    theta0)*(l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0))+(-2)* 
    l2*sin(q1+q2+theta0)*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+( 
    -1)*l2*sin(q1+q2+theta0)))+m3*(2*(l2*cos(q1+q2+theta0)+r2*cos(q1+ 
    q2+theta0)+l3*cos(q1+q2+q3+theta0))*(l1*cos(q1+theta0)+r1*cos(q1+theta0)+ 
    l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0))+2*(( 
    -1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+ 
    q2+q3+theta0))*((-1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)* 
    l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+ 
    q3+theta0))));
    
    h55 = (1/2)*(2*i2zz+2*i3zz+m2*(2*pow(l2,2)*pow(cos(q1+q2+theta0),2)+ 
    2*pow(l2,2)*pow(sin(q1+q2+theta0),2))+m3*(2*pow((l2*cos(q1+q2+theta0)+r2*cos( 
    q1+q2+theta0)+l3*cos(q1+q2+q3+theta0)),2)+2*pow(((-1)*l2*sin(q1+q2+theta0)+ 
    (-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0)),2)));
    
    h56 = (1/2)* 
    (2*i3zz+m3*(2*l3*cos(q1+q2+q3+theta0)*(l2*cos(q1+q2+theta0)+r2* 
    cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0))+(-2)*l3*sin(q1+q2+q3+theta0) 
    *((-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3* 
    sin(q1+q2+q3+theta0))));
      //////////////
    h61 = (-1)*l3*m3*sin(q1+q2+q3+theta0);
    
    h62 = l3*m3*cos( 
    q1+q2+q3+theta0);
    
    h63 = (1/2)*(2*i3zz+m3*(2*l3*cos(q1+q2+q3+theta0)*(r0x* 
    cos(theta0)+l1*cos(q1+theta0)+r1*cos(q1+theta0)+l2*cos(q1+q2+theta0)+r2* 
    cos(q1+q2+theta0)+l3*cos(q1+q2+q3+theta0)+(-1)*r0y*sin(theta0))+(-2)* 
    l3*sin(q1+q2+q3+theta0)*((-1)*r0y*cos(theta0)+(-1)*r0x*sin(theta0)+( 
    -1)*l1*sin(q1+theta0)+(-1)*r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+ 
    theta0)+(-1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))));
    
    h64 = (1/2) 
    *(2*i3zz+m3*(2*l3*cos(q1+q2+q3+theta0)*(l1*cos(q1+theta0)+r1*cos( 
    q1+theta0)+l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+q3+ 
    theta0))+(-2)*l3*sin(q1+q2+q3+theta0)*((-1)*l1*sin(q1+theta0)+(-1)* 
    r1*sin(q1+theta0)+(-1)*l2*sin(q1+q2+theta0)+(-1)*r2*sin(q1+q2+theta0)+ 
    (-1)*l3*sin(q1+q2+q3+theta0))));

    h65 = (1/2)*(2*i3zz+m3*(2*l3*cos(q1+ 
    q2+q3+theta0)*(l2*cos(q1+q2+theta0)+r2*cos(q1+q2+theta0)+l3*cos(q1+q2+ 
    q3+theta0))+(-2)*l3*sin(q1+q2+q3+theta0)*((-1)*l2*sin(q1+q2+theta0)+( 
    -1)*r2*sin(q1+q2+theta0)+(-1)*l3*sin(q1+q2+q3+theta0))));

    h66 = (1/2)*(2* 
    i3zz+m3*(2*pow(l3,2)*pow(cos(q1+q2+q3+theta0),2)+2*pow(l3,2)*pow(sin(q1+q2+q3+ 
    theta0),2)));


    
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

    // w = jacobian*h.inverse()*jacobian.transpose(); den to xreiazomaste pleon
    //isos w = jacobian.lu().solve(h) afou einai jacobian/h
    // md_b << kd_b(0,0)/pow(wn_free,2), 0, 0,
    //         0, kd_b(1,1)/pow(wn_free,2), 0, 
    //         0, 0, kd_b(2,2)/pow(wn_free,2);

   



    // rEddotdot(0) = xEddotdot;
    // rEddotdot(1) = yEddotdot;
    // rEddotdot(2) = thetaEddotdot;
    

    // e = xee - xd;
    // edot = xeedot - xddot; 
    //ta svhno giati trexo apo base controller
    /*
    e(0) = xee(0) - xd(0);
    e(1) = xee(1) - xd(1);
    e(2) = xee(2) - xd(2);
    e(3) = xc0 - xd_b(0); //base
    e(4) = yc0 - xd_b(1);
    e(5) = theta0 - xd_b(2);


    edot(0) = xeedot(0) - xddot(0);
    edot(1) = xeedot(1) - xddot(1);
    edot(2) = xeedot(2) - xddot(2);
    edot(3) = xc0dot - xd_bdot(0);
    edot(4) = yc0dot - xd_bdot(1);
    edot(5) = theta0dot - xd_bdot(2);
    */

    

    /* fact = (Eigen::MatrixXd::Identity(3, 3) - w.inverse() * md_e.inverse()) * fext +
                        w.inverse() * (jacobian * h.inverse()*c - jacobiandot * zdot) +
                        w.inverse() * md_e.inverse() * (fdes_star - (bd_e * edot) - (kd_e * e)) +
                        w.inverse() * xddotdot; //(H.colPivHouseholderQr().solve(C))) TODO:xddotdot(desired troxia), jacobiandot CHECK,zdot CHECK
                        */
    //na grapso thn sxesh pou syndeei ta torq[] me fact kai to n
    //factx, facty,nact
    
    //ta svino giati trexo apo base controller
    
    // zdot(0) = xc0dot;
    // zdot(1) = yc0dot;
    // zdot(2) = theta0dot;
    // zdot(3) = q1dot;
    // zdot(4) = q2dot;
    // zdot(5) = q3dot;

    // zddotdot(0) = xddotdot(0); //desired of end effector
    // zddotdot(1) = xddotdot(1);
    // zddotdot(2) = xddotdot(2);
    // zddotdot(3) = xd_bdotdot(0); //desired of base 
    // zddotdot(4) = xd_bdotdot(1);
    // zddotdot(5) = xd_bdotdot(2);
    

    fdes_ee = fdes*fext(0)/(fext(0)+0.00001);
    //to fdes_b einai mhden eksorismou
    // // fdes_star.head(3) = fdes_ee;
    // // fdes_star.tail(3) = fdes_b;
    fdes_star(0) = fdes_ee(0);
    fdes_star(1) = fdes_ee(1);
    fdes_star(2) = fdes_ee(2);
    fdes_star(3) = fdes_b(0);
    fdes_star(4) = fdes_b(1);
    fdes_star(5) = fdes_b(2);


    
    // fext_star.head(3) = fext;
    // fext_star.tail(3) = Eigen::VectorXd::Zero(3);
    //ta svhno logo base contorller
    // fext_star(0) = fext(0);
    // fext_star(1) = fext(1);
    // fext_star(2) = fext(2);
    // fext_star(3) = 0;
    // fext_star(4) = 0;
    // fext_star(5) = 0;




    qext = (je.transpose())*fext;

    // std::cout<<"fdes_star is: "<<fdes_star<<std::endl;

    // std::cout<<"fext_star is: "<<fext_star<<std::endl;

    // std::cout<<"qext is: "<<qext<<std::endl;

    // vec1 = jacobiandot*zdot;
    // vec2 = (zddotdot+md.inverse()*(-fext_star+fdes_star-bd*edot-kd*e)-jacobiandot*zdot);
    // vec3 = (h*jacobian.inverse())*(zddotdot+md.inverse()*(-fext_star+fdes_star-bd*edot-kd*e)-jacobiandot*zdot);

    // std::cout<<"jacobiandot*zdot is (vec1): "<<vec1<<std::endl;
    // std::cout<<"(zddotdot+md.inverse()*(-fext_star+fdes_star-bd*edot-kd*e)-jacobiandot*zdot) is (vec2): "<<vec2<<std::endl;
    // std::cout<<"(h*jacobian.inverse())*(zddotdot+md.inverse()*(-fext_star+fdes_star-bd*edot-kd*e)-jacobiandot*zdot) is (vec3): "<<vec3<<std::endl;
    // std::cout<<"c is: "<<c<<std::endl;
    // std::cout<<"qext is: "<<qext<<std::endl;
    // std::cout<<"jacobian determinant is: "<<jacobian.determinant()<<std::endl;
    // mat = h*jacobian.inverse();



    // qact=qext+c+(h*jacobian.inverse())*(zddotdot+md.inverse()*(-fext_star+fdes_star-bd*edot-kd*e)-jacobiandot*zdot);
    // qact = je.transpose()*fact;

    // torq[0] = qact(3); //torque of q1
    // torq[1] = qact(4); // of q2
    // torq[2] = qact(5); // of q3
     
}

void ImpedanceControlUpdateStep(){
    // double force_error = target_force - msg->force.z;
    // double torque_cmd = stiffness * (target_position - joint1_angle) - damping * msg->torque.z;

}

double lsTorqCalc();

double leTorqCalc();

void JointControlUpdateStep(){
  /*
	errorq[0] = q1des - q1;
	errorq[1] = q2des - q2;
	error_qdot[0] = q1dotdes - q1dot;
	error_qdot[1] = q2dotdes - q2dot;
	torq[0] = Kp*errorq[0] + Kd*error_qdot[0];
	torq[1] = Kp*errorq[1] + Kd*error_qdot[1];
    if(abs(errorq[0])<0.05 && abs(errorq[1])<0.05 && abs(error_qdot[0])<0.05 && abs(error_qdot[1])<0.05){
        reachedTarget = true;
    }
    else{
        std::cout<<"the error of q1 is: " <<errorq[0]<< " the error of q2 is: "<<errorq[1]<<" the error of q1dot is: "<<error_qdot[0]<<" the error of q2dot is: "<<error_qdot[1]<<std::endl;
    }
    */
} 

void desiredTrajectory(double t){//PROSOXH!: allagh ton indexes apo matlab se c++ stous pinakes


    s_x = a5x*pow(t,5) + a4x*pow(t,4) + a3x*pow(t,3) + a2x*pow(t,2) + a1x*t +a0x;
    s_y = a5y*pow(t,5) + a4y*pow(t,4) + a3y*pow(t,3) + a2y*pow(t,2) + a1y*t +a0y;
    s_theta = a5t*pow(t,5) + a4t*pow(t,4) + a3t*pow(t,3) + a2t*pow(t,2) + a1t*t +a0t;
    /////
    sdot_x=a1x+2*a2x*t+3*a3x*pow(t,2)+4*a4x*pow(t,3)+5*a5x*pow(t,4);
    sdot_y=a1y+2*a2y*t+3*a3y*pow(t,2)+4*a4y*pow(t,3)+5*a5y*pow(t,4);
    sdot_theta=a1t+2*a2t*t+3*a3t*pow(t,2)+4*a4t*pow(t,3)+5*a5t*pow(t,4);
    /////
    sdotdot_x=2*a2x+6*a3x*t+12*a4x*pow(t,2)+20*a5x*pow(t,3);
    sdotdot_y=2*a2y+6*a3y*t+12*a4y*pow(t,2)+20*a5y*pow(t,3);
    sdotdot_theta=2*a2t+6*a3t*t+12*a4t*pow(t,2)+20*a5t*pow(t,3);
    /////
    /*free space phase trajectory*/
    xfEd=xE_in+s_x*(xE_contact-xE_in);
    xfEddot=sdot_x*(xE_contact-xE_in);
    xfEddotdot=sdotdot_x*(xE_contact-xE_in);
    /////
    yfEd=xE_in+s_y*(yE_contact-yE_in);
    yfEddot=sdot_y*(yE_contact-yE_in);
    yfEddotdot=sdotdot_y*(yE_contact-yE_in);
    /////
    thetafEd=thetaE_in+ s_theta*(thetaE_contact-thetaE_in); //+M_PI ??
    thetafEddot=sdot_theta*(thetaE_contact-thetaE_in);
    thetafEddotdot=sdotdot_theta*(thetaE_contact-thetaE_in);
    //////
    
    
    /*contact phase trajectory*/
    //xcEd=xt+xE_contact-xt_in; //anti gia x_target=xt
    xcEd = xt-l0;
    xcEddot=xtdot;
    xcEddotdot=fext(0)/mt;
    /////
    ycEd= yt;// anti gia isos yE_contact;
    ycEddot=ytdot;//anti gia 0
    ycEddotdot=0;
    /////
    thetacEd=thetat - M_PI/2; //isos anti gia thetaE_contact;
    thetacEddot=thetatdot; //anti gia 0
    thetacEddotdot=0; 

    
    xfd(0) = xfEd;
    xfd(1) = yfEd;
    xfd(2) = thetafEd;
    xfddot(0) = xfEddot;
    xfddot(1) = yfEddot;
    xfddot(2) = thetafEddotdot;
    xfddotdot(0) = xfEddotdot;
    xfddotdot(1) = yfEddotdot;
    xfddotdot(2) = thetafEddotdot;

    xcd(0) = xcEd;
    xcd(1) = ycEd;
    xcd(2) = thetacEd;
    xcddot(0) = xcEddot;
    xcddot(1) = ycEddot;
    xcddot(2) = thetacEddotdot;
    xcddotdot(0) = xcEddotdot;
    xcddotdot(1) = ycEddotdot;
    xcddotdot(2) = thetacEddotdot;

    /*!!!DIAGNOSTICS!!!*/
    std::cout<<"current duration time is: "<<t<<std::endl;
    // std::cout<<"xfd(0) is: "<<xfd(0)<<" and xcd(0) is: "<<xcd(0)<<std::endl;
    //std::cout<<"s_x is: "<<s_x<<std::endl;
    //std::cout<<"s_y is: "<<s_y<<std::endl;
    //std::cout<<"s_theta is: "<<s_theta<<std::endl;
    //std::cout<<"a5x is: "<<a5x<<" a4x is: "<<a4x<<" a3x is: "<<a3x<<" a2x is: "<<a2x<<" a1x is: "<<a1x<<" a0x is: "<<a0x<<std::endl;
    //std::cout<<"a_matrix is: "<<a_matrix<<std::endl;
    //std::cout<<"b1_x is: "<<b1_x<<std::endl;
    //std::cout<<"a_matrix inverse is: "<<a_matrix.inverse()<<std::endl;



    /*allazo ligo se if else*/
    //xd = xfd*((abs(1-abs(fext(0))/a1))/(1+a1*abs(fext(0)))) + xcd*abs(fext(0))/(abs(fext(0))+a2);
    //xddot = xfddot*((abs(1-abs(fext(0))/a1))/(1+a1*abs(fext(0)))) + xcddot*abs(fext(0))/(abs(fext(0))+a2);
    //xddotdot = xfddotdot*((abs(1-abs(fext(0))/a1))/(1+a1*abs(fext(0)))) + xcddotdot*abs(fext(0))/(abs(fext(0))+a2);

    if(abs(fext(0))<3){
        xd = xfd;
        xddot = xfddot;
        xddotdot = xfddotdot;
    }
    else{
        xd = xcd;
        xddot = xcddot;
        xddotdot = xcddotdot;
    }

    xd_b(0) = xd(0)-s01; //x of base
    xd_b(1) = xd(1) - s02; //y of base
    xd_b(2) = xd(2); //theta of base
    ////
    xd_bdot = xddot;
    ////
    xd_bdotdot = xddotdot;



}

// void trajparams(double t){
//     double a1 = 10000;
//     double a2 = 0.0001;
//     s_x = a5x*pow(t,5) + a4x*pow(t,4) + a3x*pow(t,3) + a2x*pow(t,2) + a1x*t +a0x;
//     s_y = a5y*pow(t,5) + a4y*pow(t,4) + a3y*pow(t,3) + a2y*pow(t,2) + a1y*t +a0y;
//     s_theta = a5t*pow(t,5) + a4t*pow(t,4) + a3t*pow(t,3) + a2t*pow(t,2) + a1t*t +a0t;
//     xdf(0) = xch_in + s_x*(xt_in-xch_in); 
//     xdf(1) = ych_in + s_y*(yt_in-ych_in);
//     xdf(2) = thetach_in + s_theta*(thetat_in-thetach_in);
//     xdc(0) = xt;
//     xdc(1) = yt;
//     xdc(2) = thetat;
//     xd = xdf*((abs(1-abs(fext(0))/a1))/(1+a1*abs(fext(0)))) + xdc*abs(fext(0))/(abs(fext(0))+a2);
// }

/////////////// CALCULATION FUNCTIONS DEFINITION END////////////////////////


void diagnostics(){
    std::cout<<"//////////////////////////"<<std::endl;
    ////////////START OF DIAGNOSTICS HERE/////////////////////
    //std::cout<<"current duration time is: "<<dur_time<<std::endl;
    //std::cout<<"theta0 is: "<<theta0<<" rad"<<std::endl;
    // std::cout<<"q1 is: "<<q1<<" rad"<<std::endl;
    // std::cout<<"q2 is: "<<q2<<" rad"<<std::endl;
    // std::cout<<"q3 is: "<<q3<<" rad"<<std::endl;
    // std::cout<<"fext_x is: "<<fext(0)<<" N "<<std::endl;
    // std::cout<<"fext_y is: "<<fext(1)<<" N "<<std::endl;
    // std::cout<<"xfd_x is: "<<xfd(0)<<" and xfd_y is: "<<xfd(1)<<std::endl;
    // std::cout<<"xcd_x is: "<<xcd(0)<<" and xcd_y is: "<<xcd(1)<<std::endl;
    // std::cout<<"xd_x is: "<< xd(0) <<std::endl;-619501
    // std::cout<<"xd_y is: "<< xd(1) <<std::endl;
    // std::cout<<"xd_theta is: "<< xd(2) <<std::endl;
    // std::cout<<"fact_x is: "<<fact(0)<<" N " <<std::endl;
    // std::cout<<"fact_y is: "<<fact(1)<<" N " <<std::endl;
    // std::cout<<"fact_t is: "<<fact(2)<<" Nm " <<std::endl;
    // std::cout<<"je.T is:"<<je.transpose()<<std::endl;

   
    // std::cout<<"q1 torque is: "<<qact(3) <<" Nm "<<std::endl;
    // std::cout<<"q2 torque is: "<<qact(4) <<" Nm "<<std::endl;
    // std::cout<<"q3 torque is: "<<qact(5) <<" Nm "<<std::endl;
    // std::cout<<"ee_x is:  "<<ee_x <<" m  "<<std::endl;
    // std::cout<<"ee_y is:  "<<ee_y <<" m  "<<std::endl;
    // std::cout<<"xee is: "<<xee<<std::endl;
    // std::cout<<"ee x is: "<<xee(0)<<" m"<<std::endl;
    // std::cout<<"ee y is: "<<xee(1)<<" m"<<std::endl;
    // std::cout<<"ee theta is: "<<xee(2)<<" rad"<<std::endl;
    // std::cout<<"target x is: "<<xt<<" m"<<std::endl;
    // std::cout<<"target y is: "<<yt<<" m"<<std::endl;
    // std::cout<<"target theta is: "<<thetat<<" rad"<<std::endl;

    std::cout<<"fext x is: "<<fext(0)<<" N"<<std::endl;
    std::cout<<"fext y is: "<<fext(1)<<" N"<<std::endl;
    std::cout<<"next  is: "<<fext(2)<<" Nm"<<std::endl;
    // std::cout<<"error x is: "<<e(0)<<" m "<<std::endl;
    // std::cout<<"error y is: "<<e(1)<<" m "<<std::endl;
    // std::cout<<"error theta is: "<<e(2)<<" rad "<<std::endl;
    // std::cout<<"fx is: "<<qact(0)<<" N "<<std::endl;
    // std::cout<<"fy is: "<<qact(1)<<" N "<<std::endl;
    // std::cout<<"RW torque is: "<<qact(2)<<" Nm "<<std::endl;
    // std::cout<<"t1 (shoulder torque) is: "<<qact(3)<<" Nm "<<std::endl;
    // std::cout<<"t2 (elbow torque) is: "<<qact(4)<<" Nm "<<std::endl;
    // std::cout<<"t3 (wrist torque) is: "<<qact(5)<<" Nm "<<std::endl;

    std::cout<<"jacobian is: "<<jacobian<<std::endl;
    std::cout<<"h is: "<<h<<std::endl;
    std::cout<<"jacobian.inverse is: "<<jacobian.inverse()<<std::endl;
    std::cout<<"h*jacobian.inverse() is: "<<mat<<std::endl;
    std::cout<<"qact calculated is: "<<qact<<std::endl;


    // std::cout<<"h is: "<<h <<std::endl;
    // std::cout<<"je is: "<<je <<std::endl;
    // std::cout<<"jacobian is: "<<jacobian<<std::endl;
    // std::cout<<"jacobian dot is: "<<jacobiandot<<std::endl;


    // std::cout<<"fact x is: "<<fact(0)<<" N"<<std::endl;
    // std::cout<<"fact y is: "<<fact(1)<<" N"<<std::endl;
    // std::cout<<"nact  is: "<<fact(2)<<" Nm"<<std::endl;


    ////////////END OF DIAGNOSTICS HERE/////////////////

    std::cout<<" "<<std::endl;
}