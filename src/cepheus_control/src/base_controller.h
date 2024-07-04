#include "includes.h"
#include "variables.h"


void basePDcontroll(){
    double ex,ey,eth;
    double exdot, eydot, ethdot;

    //we take the desired xdes,ydes,tdes for each time step
    //from desiredTrajectory: xfEd,yfEd,thetafEd and dots.
    
    /*for simple PD: */
    ex = (xt-l3-r3-0.01)- ee_x; //xfEd - ee_x;
    ey = yt-ee_y; //yfEd - ee_y; 
    eth = thetat - thetach; //thetafEd
    exdot = xtdot - xeedot(0); //xfEddot
    eydot = ytdot - xeedot(1); //yfEddot
    ethdot = thetatdot - xeedot(2); //thetafEddotdot
    /*end of simple PD*/

    /*for polynomial trajectory: */
    // ex = xstep- ee_x; //xfEd - ee_x;
    // ey = ystep-ee_y; //yfEd - ee_y; 
    // eth = thstep - thetach; //thetafEd
    // exdot = xstepdot - xeedot(0); //xfEddot
    // eydot = ystepdot - xeedot(1); //yfEddot
    // ethdot = thstepdot - xeedot(2); //thetafEddotdot
    /*end of polynomial trajectory*/

    std::cout<<"///////////////////"<<std::endl;
    std::cout<<"yt is: "<<yt<<std::endl;
    std::cout<<"ee_y is: "<<ee_y<<std::endl;
    std::cout<<"xt is: "<<xt<<std::endl;
    std::cout<<"ee_x is: "<<ee_x<<std::endl;
    std::cout<<"xtdot is: "<<xtdot<<std::endl;
    std::cout<<"ytdot is: "<<ytdot<<std::endl;
    // std::cout<<"error x is: "<<ex<<std::endl;
    // std::cout<<"error y is: "<<ey<<std::endl;
    // std::cout<<"error x dot is: "<<exdot<<std::endl;
    // std::cout<<"error y dot is: "<<eydot<<std::endl;
    std::cout<<" "<<std::endl;

    if(abs(ex)<0.005 && abs(ey)<0.005 && abs(exdot)<0.005 && abs(eydot)<0.005 && abs(eth)<0.005 && abs(ethdot)<0.005){
        reachedTarget = true;
        // fx = fy = ns = 0;
        return ; 
    }
    fx = kprop*ex + kder*exdot;
    fy = kprop*ey +kder*eydot;
    ns = kprop*eth + kder*ethdot;

    std::cout<<"fx is: "<<fx<<std::endl;
    std::cout<<"fy is: "<<fy<<std::endl;


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

void calculateTrajecotryPolynomials(double tf){
    Eigen::MatrixXd eq_matrix(3,3);
    Eigen::VectorXd eq_bscale(3);
    
    
    eq_matrix << pow(tf,3), pow(tf,4), pow(tf,5),
                3*pow(tf,3), 4*pow(tf,3), 5*pow(tf,4),
                6*tf, 12*pow(tf,2), 20*pow(tf,3); 
    
    eq_bscale << 1, 0, 0;

    Eigen::VectorXd res = eq_matrix.colPivHouseholderQr().solve(eq_bscale);

    a0 = a1 = a2 = 0; //from paper calculations, for t0 = 0
    a3 = res(0);
    a4 = res(1);
    a5 = res(2);
}

void baseTrajectory(double t){
    double s,sdot;

    s = a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5);
    sdot = a1 + 2*a2*t + 3*a3*pow(t,2) + 4*a4*pow(t,3) + 5*a5*pow(t,4);

    xstep = xE_in + s*(xt_in-xE_in);
    ystep = yE_in + s*(yt_in - yE_in);
    thstep = thetaE_in + s*(thetat_in - thetaE_in);

    xstepdot = xE_in + sdot*(xt_in-xE_in);
    ystepdot = yE_in + sdot*(yt_in - yE_in);
    thstepdot = thetaE_in + sdot*(thetat_in - thetaE_in);

}