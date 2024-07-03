#include "includes.h"
#include "variables.h"


void basePDcontroll(){
    double ex,ey,eth;
    double exdot, eydot, ethdot;

    //we take the desired xdes,ydes,tdes for each time step
    //from desiredTrajectory: xfEd,yfEd,thetafEd and dots.

    ex = (xt-l3-r3-0.01)- ee_x; //xfEd - ee_x;
    ey = yt-ee_y; //yfEd - ee_y; 
    eth = thetat - thetach; //thetafEd
    exdot = xtdot - xeedot(0); //xfEddot
    eydot = ytdot - xeedot(1); //yfEddot
    ethdot = thetatdot - xeedot(2); //thetafEddotdot
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