#include "includes.h"
#include "variables.h"


void basePDcontroll(){
    double ex,ey,eth;
    double exdot, eydot, ethdot;

    //we take the desired xdes,ydes,tdes for each time step
    //from desiredTrajectory: xfEd,yfEd,thetafEd and dots.
    
    /*for simple PD: */
    // ex = (xt-l3-r3-0.01)- ee_x; //xfEd - ee_x;
    // ey = yt-ee_y; //yfEd - ee_y; 
    // eth = thetat - thetach; //thetafEd
    // exdot = xtdot - xeedot(0); //xfEddot
    // eydot = ytdot - xeedot(1); //yfEddot
    // ethdot = thetatdot - xeedot(2); //thetafEddotdot
    /*end of simple PD*/

    /*for polynomial trajectory: */
    ex = xstep- ee_x; //xfEd - ee_x;
    ey = ystep-ee_y; //yfEd - ee_y; 
    eth = thstep - thetach; //thetafEd
    exdot = xstepdot - xeedot(0); //xfEddot
    eydot = ystepdot - xeedot(1); //yfEddot
    ethdot = thstepdot - xeedot(2); //thetafEddotdot
    /*end of polynomial trajectory*/

    // std::cout<<"///////////////////"<<std::endl;
    // std::cout<<"yt is: "<<yt<<std::endl;
    // std::cout<<"ee_y is: "<<ee_y<<std::endl;
    // std::cout<<"xt is: "<<xt<<std::endl;
    // std::cout<<"ee_x is: "<<ee_x<<std::endl;
    // std::cout<<"xtdot is: "<<xtdot<<std::endl;
    // std::cout<<"ytdot is: "<<ytdot<<std::endl;

    // std::cout<<"error x is: "<<ex<<std::endl;
    // std::cout<<"error y is: "<<ey<<std::endl;
    // std::cout<<"error x dot is: "<<exdot<<std::endl;
    // std::cout<<"error y dot is: "<<eydot<<std::endl;
    std::cout<<" "<<std::endl;

    // if(abs(ex)<0.005 && abs(ey)<0.005 && abs(exdot)<0.005 && abs(eydot)<0.005 && abs(eth)<0.005 && abs(ethdot)<0.005){
    //     reachedTarget = true;
    //     // fx = fy = ns = 0;
    //     return ; 
    // } //na to ftiakso kalytera
    fx = kprop*ex + kder*exdot;
    fy = kprop*ey +kder*eydot;
    ns = kprop*eth + kder*ethdot;

    std::cout<<"fx is: "<<fx<<std::endl;
    std::cout<<"fy is: "<<fy<<std::endl;

    base_wrench.force.x = fx;  //fx;
    base_wrench.force.y = fy;  //fy;
    base_wrench.torque.z = ns; //ns;

    msg_xd_x.data = xstep;
    msg_xd_y.data = ystep;
    msg_xd_theta.data = thstep;
    msg_xt_x.data = xt_in; //ta evala sthn callback synarthsh
    msg_xt_y.data = yt_in;
    msg_xt_theta.data = thetat_in;
    msg_xee_x.data = ee_x;
    msg_xee_y.data = ee_y;
    msg_xee_theta.data = thetach;


}

void filter(){
    for(int i=0; i<5; i++){
        if(qact(i)>20){
            qact(i) = 20;
            }
        else if(qact(i) < -20){
            qact(i) = -20;
            }
}
}



void baseTrajectory(double t, double tf){
    double s,sdot, sdotdot;

    s = a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
    sdot = a1 + 2*a2*t + 3*a3*pow(t,2) + 4*a4*pow(t,3) + 5*a5*pow(t,4);
    sdotdot = 2*a2 + 6*a3*t + 12*a4*pow(t,2) + 20*a5*pow(t,3);

    if(t<=tf){
        xstep = xE_in + s*(xt_in - xE_in);
        ystep = yE_in + s*(yt_in - yE_in);
        thstep = thetaE_in + s*(thetat_in - thetaE_in);

        xstepdot = sdot*(xt_in-xE_in);
        ystepdot = sdot*(yt_in - yE_in);
        thstepdot = sdot*(thetat_in - thetaE_in);

        xstepdotdot = sdotdot*(xt_in-xE_in);
        ystepdotdot = sdotdot*(yt_in - yE_in);
        thstepdotdot = sdotdot*(thetat_in - thetaE_in);
    }
    else{
        xstep = xt_in;
        ystep = yt_in;
        thstep = thetat_in;
        xstepdot = 0;
        ystepdot = 0;
        thstepdot = 0;
        xstepdotdot = 0;
        ystepdotdot = 0;
        thstepdotdot = 0;
    }

    std::cout<<"////////////////////"<<std::endl;
    std::cout<<"time is: "<<t<<std::endl;

}

void calculateQ(){
    Eigen::VectorXd x(6);
    x << ee_x, ee_y, thetach, xc0, yc0, theta0;
    Eigen::VectorXd xdot(6);
    xdot << xeedot(0), xeedot(1), xeedot(2), xc0dot, yc0dot, theta0dot;
    Eigen::VectorXd xd(6);
    xd   << xstep , ystep, thstep, xstep-s01, ystep - s02, thstep;
    Eigen::VectorXd xddot(6);
    xddot << xstepdot, ystepdot, thstepdot, xstepdot, ystepdot, thstepdot;
    Eigen::VectorXd xddotdot(6);
    xddotdot <<xstepdotdot, ystepdotdot, thstepdotdot, xstepdotdot, ystepdotdot, thstepdotdot;
    // Eigen::MatrixXd temp(6,6);
    // Eigen::MatrixXd tempinv(6,6);
    Eigen::VectorXd zdot(6);
    /*allazo to e apo xd-x se x -xd ara allazei o nomos elegxou u*/
    Eigen::VectorXd e =  x - xd;
    Eigen::VectorXd edot = xdot - xddot;
    Eigen::VectorXd u = xddotdot - kprop_mb*e - kder_mb*edot;



    zdot << xc0dot, yc0dot, theta0dot, q1dot, q2dot, q3dot;
    

    // Eigen::MatrixXd temp = jacobian*(h.inverse());
    // Eigen::MatrixXd tempinv = temp.inverse();
    Eigen::MatrixXd temp = h*(jacobian.inverse());
    

    edot = xddot - xdot;
    // u=xddotdot + kprop*e + kder*edot;
    // qact = h*u +c; //oxi toso aplo
    Eigen::VectorXd q = c + temp*(u - jacobiandot*zdot);
    // Eigen::VectorXd q = c + temp*(xddotdot+ 1*(-kder*edot - kprop*e) - jacobiandot*zdot);


    //meiktos controller, den doulevei kala
    // Eigen::VectorXd q = qext + c + temp*(xddotdot + md.inverse()*(-fext_star + fdes_star -bd*edot -kd*e)-jacobiandot*zdot);


    // std::cout <<" c is: "<<c<<std::endl;
    // std::cout <<" h*j^-1 is: "<<temp<<std::endl;
    // std::cout << "e is: "<< e<<std::endl;
    std::cout << "q is: "<<q<<std::endl;

    // std::cout<<"[Q calculator]: fx is: "<<q(0)<<std::endl;
    // std::cout<<"[Q calculator]: fy is: "<<q(1)<<std::endl;
    // std::cout<<"[Q calculator]: RW is: "<<q(2)<<std::endl;

    base_wrench.force.x = q(0);  //fx;
    base_wrench.force.y = q(1);  //fy;
    base_wrench.torque.z = q(2); //ns;
    // msg_RW.data = q(2); // to bgazo kai evala to wrench
	msg_LS.data = q(3);
	msg_LE.data = q(4);
	msg_LW.data = q(5);

    msg_xd_x.data = xd(0);
    msg_xd_y.data = xd(1);
    msg_xd_theta.data = xd(2);
    msg_xt_x.data = xt_in; //ta evala sthn callback synarthsh
    msg_xt_y.data = yt_in;
    msg_xt_theta.data = thetat_in;
    msg_xee_x.data = x(0);
    msg_xee_y.data = x(1);
    msg_xee_theta.data = x(2);
}