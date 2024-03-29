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
    m0 = 54; //physical parameters from alex paper
    r0 = 0.16;
    l1 = 0.26931;
    r1 = 0.100069;
    m1 = 0.2314;
    l2 = 0.143;
    r2 = 0.143;
    m2 = 0.08797;
    l3 = 0.17732;
    r3 = 0.09768;
    m3 = 6;

    M = m0 + m1 + m2 + m3;
    // a = r0*m0/M;
    // b = (l1*m0 + (m0+m1)*r1)/M;
    // c = (l2*(m0 + m1) + (m0 + m1 + m2)*r2)/M;
    // d = r3 + l3*(m0 + m1 + m2)/M;

    xch_c << 0, 0, 0;
    xdf << 0, 0, 0;
    xdc << 0, 0, 0;
    xee << 0, 0, 0;
    xeedot << 0, 0, 0;

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

        


}

void calculateStep(){  //calculate stuff in each iteration
    j16 = -l3*sin(theta0+q1+q2+q3);
    j15 = j16 - l2*sin(theta0+q1+q2);
    j14 = j15 - l1*sin(theta0+q1);
    j13 = j14 - l0*sin(theta0);

    j26 = l3*cos(theta0+q1+q2+q3);
    j25 = j26 + l2*cos(theta0+q1+q2);
    j24 = j25 + l1*cos(theta0+q1);
    j23 = j24 + l0*cos(theta0);

    jacobian(1,3) = j13;
    jacobian(1,4) = j14;
    jacobian(1,5) = j15;
    jacobian(1,6) = j16;
    jacobian(2,3) = j23;
    jacobian(2,4) = j24;
    jacobian(2,5) = j25;
    jacobian(2,6) = j26;

    j16dot = -l3*cos(theta0+q1+q2+q3)*(theta0dot+q1dot+q2dot+q3dot);
    j15dot = j16dot - l2*cos(theta0+q1+q2)*(theta0dot+q1dot+q2dot);
    j14dot = j15dot - l1*cos(theta0+q1)*(theta0dot+q1dot);
    j13dot = j14dot - l0*cos(theta0)*theta0dot;

    j26dot = -l3*sin(theta0+q1+q2+q3)*(theta0dot+q1dot+q2dot+q3dot);
    j25dot = j26dot - l2*sin(theta0+q1+q2)*(theta0dot+q1dot+q2dot);
    j24dot = j25dot - l1*sin(theta0+q1)*(theta0dot+q1dot);
    j23dot = j24dot - l0*sin(theta0)*theta0dot;
    
    jacobiandot(1,3) = j13dot;
    jacobiandot(1,4) = j14dot;
    jacobiandot(1,5) = j15dot;
    jacobiandot(1,6) = j16dot;
    jacobiandot(2,3) = j23dot;
    jacobiandot(2,4) = j24dot;
    jacobiandot(2,5) = j25dot;
    jacobiandot(2,6) = j26dot;

    z(0) = xc0;
    z(1) = yc0;
    z(2) = theta0;
    z(3) = q1;
    z(4) = q2;
    z(5) = q3;

    zdot(0) = xc0dot;
    zdot(1) = yc0dot;
    zdot(2) = theta0dot;
    zdot(3) = q1dot;
    zdot(4) = q2dot;
    zdot(5) = q3dot;

    
    
    c11 = -(m1 + m2 + m3) * r0 * theta0dot * theta0dot * cos(theta0)
            - (l1 * (m1 + m2 + m3) + (m2 + m3) * r1) * (q1dot + theta0dot) * (q1dot + theta0dot) * cos(q1 + theta0)
            - (l2 * (m2 + m3) + m3 * r2) * (q1dot + q2dot + theta0dot) * (q1dot + q2dot + theta0dot) * cos(q1 + q2 + theta0)
            - l3 * m3 * (q1dot + q2dot + q3dot + theta0dot) * (q1dot + q2dot + q3dot + theta0dot) * cos(q1 + q2 + q3 + theta0);
    
    c21 = -(m1 + m2 + m3) * r0 * theta0dot * theta0dot * sin(theta0)
            - (l1 * (m1 + m2 + m3) + (m2 + m3) * r1) * (q1dot + theta0dot) * (q1dot + theta0dot) * sin(q1 + theta0)
            - (l2 * (m2 + m3) + m3 * r2) * (q1dot + q2dot + theta0dot) * (q1dot + q2dot + theta0dot) * sin(q1 + q2 + theta0)
            - l3 * m3 * (q1dot + q2dot + q3dot + theta0dot) * (q1dot + q2dot + q3dot + theta0dot) * sin(q1 + q2 + q3 + theta0);
    
    c31 = -q1dot * r0 * (l1 * (m1 + m2 + m3) + (m2 + m3) * r1) * (q1dot + 2 * theta0dot) * sin(q1)
            + l3 * m3 * cos(q3) * (- (q1dot + q2dot + q3dot) * r0 * (q1dot + q2dot + q3dot + 2 * theta0dot) * cos(q2) * sin(q1)
                                    - ((q2dot + q3dot) * (l1 + r1) * (2 * q1dot + q2dot + q3dot + 2 * theta0dot)
                                       + (q1dot + q2dot + q3dot) * r0 * (q1dot + q2dot + q3dot + 2 * theta0dot) * cos(q1)) * sin(q2))
            + (l2 * (m2 + m3) + m3 * r2) * ( - q2dot * (l1 + r1) * (2 * q1dot + q2dot + 2 * theta0dot) * sin(q2)
                                              - (q1dot + q2dot) * r0 * (q1dot + q2dot + 2 * theta0dot) * sin(q1 + q2))
            - l3 * m3 * (q3dot * (l2 + r2) * (2 * q1dot + 2 * q2dot + q3dot + 2 * theta0dot)
                          + (q2dot + q3dot) * (l1 + r1) * (2 * q1dot + q2dot + q3dot + 2 * theta0dot) * cos(q2)
                          + (q1dot + q2dot + q3dot) * r0 * (q1dot + q2dot + q3dot + 2 * theta0dot) * cos(q1 + q2)) * sin(q3);
    
    c41 = r0 * theta0dot * theta0dot * (l1 * (m1 + m2 + m3) + (m2 + m3) * r1 + l3 * m3 * cos(q2) * cos(q3)) * sin(q1)
            - l3 * m3 * ((q2dot + q3dot) * (l1 + r1) * (2 * q1dot + q2dot + q3dot + 2 * theta0dot)
                          - r0 * theta0dot * theta0dot * cos(q1)) * cos(q3) * sin(q2)
            + (l2 * (m2 + m3) + m3 * r2) * ( - q2dot * (l1 + r1) * (2 * q1dot + q2dot + 2 * theta0dot) * sin(q2)
                                              + r0 * theta0dot * theta0dot * sin(q1 + q2))
            - l3 * m3 * (q3dot * (l2 + r2) * (2 * q1dot + 2 * q2dot + q3dot + 2 * theta0dot)
                          + (q2dot + q3dot) * (l1 + r1) * (2 * q1dot + q2dot + q3dot + 2 * theta0dot) * cos(q2)
                          - r0 * theta0dot * theta0dot * cos(q1 + q2)) * sin(q3);
    
    c51 = (l2 * (m2 + m3) + m3 * r2 + l3 * m3 * cos(q3)) 
            * ((l1 + r1) * (q1dot + theta0dot) * (q1dot + theta0dot) * sin(q2) 
            + r0 * theta0dot * theta0dot * sin(q1 + q2)) 
            + l3 * m3 * ((-1) * q3dot * (l2 + r2) * (2 * q1dot + 2 * q2dot + q3dot + 2 * theta0dot) 
            + (l1 + r1) * (q1dot + theta0dot) * (q1dot + theta0dot) * cos(q2) 
            + r0 * theta0dot * theta0dot * cos(q1 + q2)) * sin(q3);
    
    c61 = l3 * m3 * ((l2 + r2) * (q1dot + q2dot + theta0dot) * (q1dot + q2dot + theta0dot) * sin(q3) 
            + (l1 + r1) * (q1dot + theta0dot) * (q1dot + theta0dot) * sin(q2 + q3) 
            + r0 * theta0dot * theta0dot * sin(q1 + q2 + q3));
    
    c(0) = c11;
    c(1) = c21;
    c(2) = c31;
    c(3) = c41;
    c(4) = c51;
    c(5) = c61;

    h11 = m1 + m2 + m3 + mb;
    h12 = 0;
    h13 = (-1) * (m1 + m2 + m3) * r0 * sin(theta0) + (-1) * (l1 * (m1 + m2 + m3) + (m2 + m3) * r1) * sin(q1 + theta0) + (-1) * (l2 * (m2 + m3) + m3 * r2) * sin(q1 + q2 + theta0) + (-1) * l3 * m3 * sin(q1 + q2 + q3 + theta0);
    h14 = (-1)*(l1*(m1+m2+m3)+(m2+m3)*r1)*sin(q1+theta0)+(-1)*(l2*(m2+m3)+m3*r2)*sin(q1+q2+theta0)+(-1)*l3*m3*sin(q1+q2+q3+theta0);
    h15 = (-1) * (l2 * (m2 + m3) + m3 * r2) * sin(q1 + q2 + theta0) + (-1) * l3 * m3 * sin(q1 + q2 + q3 + theta0);
    h16 = (-1) * l3 * m3 * sin(q1 + q2 + q3 + theta0); 
    ///////////////
    h21 = 0;
    h22 = m1 + m2 + m3 + mb;
    h23 = (m1 + m2 + m3) * r0 * cos(theta0) + (l1 * (m1 + m2 + m3) + (m2 + m3) * r1) * cos(q1 + theta0) + (l2 * (m2 + m3) + m3 * r2) * cos(q1 + q2 + theta0) + l3 * m3 * cos(q1 + q2 + q3 + theta0); 
    h24 = (l1 * (m1 + m2 + m3) + (m2 + m3) * r1) * cos(q1 + theta0) + (l2 * (m2 + m3) + m3 * r2) * cos(q1 + q2 + theta0) + l3 * m3 * cos(q1 + q2 + q3 + theta0);
    h25 = (l2 * (m2 + m3) + m3 * r2) * cos(q1 + q2 + theta0) + l3 * m3 * cos(q1 + q2 + q3 + theta0);
    h26 = l3 * m3 * cos(q1 + q2 + q3 + theta0);
    ///////////////
    h31 = (-1) * (m1 + m2 + m3) * r0 * sin(theta0) + (-1) * (l1 * (m1 + m2 + m3) + (m2 + m3) * r1) * sin(q1 + theta0) + (-1) * (l2 * (m2 + m3) + m3 * r2) * sin(q1 + q2 + theta0) + (-1) * l3 * m3 * sin(q1 + q2 + q3 + theta0);
    h32 = (m1 + m2 + m3) * r0 * cos(theta0) + (l1 * (m1 + m2 + m3) + (m2 + m3) * r1) * cos(q1 + theta0) + (l2 * (m2 + m3) + m3 * r2) * cos(q1 + q2 + theta0) + l3 * m3 * cos(q1 + q2 + q3 + theta0);
    h33 = i1zz + i2zz + i3zz + ibzz + l2 * l2 * (m2 + m3) + l1 * l1 * (m1 + m2 + m3) + (m1 + m2) * r0 * r0 + 2 * l1 * (m2 + m3) * r1 + m2 * r1 * r1 + 2 * l2 * m3 * r2 + m3 * (l3 * l3 + r0 * r0 + r1 * r1 + r2 * r2) + 2 * (r0 * (l1 * (m1 + m2 + m3) + (m2 + m3) * r1) * cos(q1) + (l2 * (m2 + m3) + m3 * r2) * ((l1 + r1) * cos(q2) + r0 * cos(q1 + q2)) + l3 * m3 * (l2 + r2 + (l1 + r1) * cos(q2) + r0 * cos(q1 + q2)) * cos(q3) + (-1) * l3 * m3 * ((l1 + r1) * sin(q2) + r0 * sin(q1 + q2)) * sin(q3));
    h34 = i1zz + i2zz + i3zz + l2 * l2 * (m2 + m3) + l1 * l1 * (m1 + m2 + m3) + 2 * l1 * (m2 + m3) * r1 + m2 * r1 * r1 + 2 * l2 * m3 * r2 + m3 * (l3 * l3 + r1 * r1 + r2 * r2) + r0 * (l1 * (m1 + m2 + m3) + (m2 + m3) * r1) * cos(q1) + (l2 * (m2 + m3) + m3 * r2) * (2 * (l1 + r1) * cos(q2) + r0 * cos(q1 + q2)) + l3 * m3 * (2 * (l2 + r2 + (l1 + r1) * cos(q2)) + r0 * cos(q1 + q2)) * cos(q3) + (-1) * l3 * m3 * (2 * (l1 + r1) * sin(q2) + r0 * sin(q1 + q2)) * sin(q3);
    h35 = i2zz+i3zz+pow(l2,2)*(m2+m3)+2*l2*m3*r2+m3*(pow(l3,2)+pow(r2,2))+(l1+r1)*(l2*(m2+m3)+m3*r2)*cos(q2)+r0*(l2*(m2+m3)+m3*r2)*cos(q1+q2)+l3*m3*(2*(l2+r2)*cos(q3)+(l1+r1)*cos(q2+q3)+r0*cos(q1+q2+q3));
    h36 = i3zz+l3*l3*m3+l3*m3*((l2+r2)*cos(q3)+(l1+r1)*cos(q2+q3)+r0*cos(q1+q2+q3));
    //////////////
    h41 = (-1)*(l1*(m1+m2+m3)+(m2+m3)*r1)*sin(q1+theta0)+(-1)*(l2*(m2+m3)+m3*r2)*sin(q1+q2+theta0)+(-1)*l3*m3*sin(q1+q2+q3+theta0);
    h42 = (l1*(m1+m2+m3)+(m2+m3)*r1)*cos(q1+theta0)+(l2*(m2+m3)+m3*r2)*cos(q1+q2+theta0)+l3*m3*cos(q1+q2+q3+theta0);
    h43 = i1zz+i2zz+i3zz+l2*l2*(m2+m3)+l1*l1*(m1+m2+m3)+2*l1*(m2+m3)*r1+m2*r1*r1+2*l2*m3*r2+m3*(l3*l3+r1*r1+r2*r2)+r0*(l1*(m1+m2+m3)+(m2+m3)*r1)*cos(q1)+(l2*(m2+m3)+m3*r2)*(2*(l1+r1)*cos(q2)+r0*cos(q1+q2))+l3*m3*(2*(l2+r2+(l1+r1)*cos(q2))+r0*cos(q1+q2))*cos(q3)+(-1)*l3*m3*(2*(l1+r1)*sin(q2)+r0*sin(q1+q2))*sin(q3);
    h44 = i1zz+i2zz+i3zz+l2*l2*m2+l2*l2*m3+l3*l3*m3+l1*l1*(m1+m2+m3)+2*l1*(m2+m3)*r1+m2*r1*r1+m3*r1*r1+2*l2*m3*r2+m3*r2*r2+2*(l1+r1)*(l2*(m2+m3)+m3*r2)*cos(q2)+2*l3*m3*((l2+r2)*cos(q3)+(l1+r1)*cos(q2+q3));
    h45 = i2zz+i3zz+l2*l2*(m2+m3)+2*l2*m3*r2+m3*(l3*l3+r2*r2)+(l1+r1)*(l2*(m2+m3)+m3*r2)*cos(q2)+l3*m3*(2*(l2+r2)*cos(q3)+(l1+r1)*cos(q2+q3));
    h46 = i3zz+l3*l3*m3+l3*m3*(l2+r2)*cos(q3)+l3*m3*(l1+r1)*cos(q2+q3);
    //////////////
    h51 = (-1)*(l2*(m2+m3)+m3*r2)*sin(q1+q2+theta0)+(-1)*l3*m3*sin(q1+q2+q3+theta0);
    h52 = (l2*(m2+m3)+m3*r2)*cos(q1+q2+theta0)+l3*m3*cos(q1+q2+q3+theta0);
    h53 = i2zz+i3zz+pow(l2,2)*(m2+m3)+2*l2*m3*r2+m3*(l3*l3+r2*r2)+(l1+r1)*(l2*(m2+m3)+m3*r2)*cos(q2)+r0*(l2*(m2+m3)+m3*r2)*cos(q1+q2)+l3*m3*(2*(l2+r2)*cos(q3)+(l1+r1)*cos(q2+q3)+r0*cos(q1+q2+q3));
    h54 = i2zz+i3zz+l2*l2*(m2+m3)+2*l2*m3*r2+m3*(l3*l3+r2*r2)+(l1+r1)*(l2*(m2+m3)+m3*r2)*cos(q2)+l3*m3*(2*(l2+r2)*cos(q3)+(l1+r1)*cos(q2+q3));
    h55 = i2zz+i3zz+l2*l2*(m2+m3)+2*l2*m3*r2+m3*(l3*l3+r2*r2)+2*l3*m3*(l2+r2)*cos(q3);
    h56 = i3zz+l3*l3*m3+l3*m3*(l2+r2)*cos(q3);
    //////////////
    h61 = (-1)*l3*m3*sin(q1+q2+q3+theta0);
    h62 = l3*m3*cos(q1+q2+q3+theta0);
    h63 = i3zz+l3*l3*m3+l3*m3*((l2+r2)*cos(q3)+(l1+r1)*cos(q2+q3)+r0*cos(q1+q2+q3));
    h64 = i3zz+l3*l3*m3+l3*m3*(l2+r2)*cos(q3)+l3*m3*(l1+r1)*cos(q2+q3);
    h65 = i3zz+l3*l3*m3+l3*m3*(l2+r2)*cos(q3);
    h66 = i3zz+l3*l3*m3;

    
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

    w = jacobian*h.inverse()*jacobian.transpose();
    md_b << kd_b(0,0)/pow(wn_free,2), 0, 0,
            0, kd_b(1,1)/pow(wn_free,2), 0, 
            0, 0, kd_b(2,2)/pow(wn_free,2);

    bd_e = 2*z_free*wn_free*md_e;
    bd_b = 2*z_free*wn_free*md_b;
    bd.topLeftCorner(3,3) = bd_e;
    bd.topRightCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    bd.bottomLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    bd.bottomRightCorner(3,3) = bd_b;

    fdes_star = fdes*fext(0)/(fext(0)+0.01);


    md.topLeftCorner(3,3) = md;
    md.topRightCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    md.bottomLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    md.bottomRightCorner(3,3) = md_b;

    rEddotdot(0) = xEddotdot;
    rEddotdot(1) = yEddotdot;
    rEddotdot(2) = thetaEddotdot;

    e = xee - xd;
    edot = xeedot - xddot; 


    

    fact = (Eigen::MatrixXd::Identity(3, 3) - w.inverse() * md.inverse()) * fext +
                        w.inverse() * (jacobian * h.inverse()*c - jacobiandot * zdot) +
                        w.inverse() * md.inverse() * (fdes_star - (bd * edot) - (kd * e)) +
                        w.transpose().inverse() * rEddotdot; //(H.colPivHouseholderQr().solve(C))) TODO:xddotdot(desired troxia), jacobiandot CHECK,zdot CHECK

    //na grapso thn sxesh pou syndeei ta torq[] me fact kai to n
    //factx, facty,nact
    je11 = 1;
    je12 = 0;
    je13 = (-1)*r0*sin(theta0)+(-1)*(l1+r1)*sin(q1+theta0)+(-1)*(l2+r2)*sin(q1+q2+theta0)+(-1)*(l3+r3)*sin(q1+q2+q3+theta0);
    je14 = ((-1)*l1+(-1)*r1)*sin(q1+theta0)+(-1)*(l2+r2)*sin(q1+q2+theta0)+(-1)*(l3+r3)*sin(q1+q2+q3+theta0);
    je15 = ((-1)*l2+(-1)*r2)*sin(q1+q2+theta0)+(-1)*(l3+r3)*sin(q1+q2+q3+theta0);
    je16 = ((-1)*l3+(-1)*r3)*sin(q1+q2+q3+theta0); 
    ////////////
    je21 = 0;
    je22 = 1;
    je23 = r0*cos(theta0)+(l1+r1)*cos(q1+theta0)+(l2+r2)*cos(q1+q2+theta0)+(l3+r3)*cos(q1+q2+q3+theta0);
    je24 = (l1+r1)*cos(q1+theta0)+(l2+r2)*cos(q1+q2+theta0)+(l3+r3)*cos(q1+q2+q3+theta0);
    je25 = (l2+r2)*cos(q1+q2+theta0)+(l3+r3)*cos(q1+q2+q3+theta0);
    je26 = (l3+r3)*cos(q1+q2+q3+theta0);
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

    qact = je.transpose()*fact;

    torq[0] = qact(3); //torque of q1
    torq[1] = qact(4); // of q2
    torq[2] = qact(5); // of q3
     
}

void ImpedanceControlUpdateStep(){
    // double force_error = target_force - msg->force.z;
    // double torque_cmd = stiffness * (target_position - joint1_angle) - damping * msg->torque.z;

}

double lsTorqCalc();

double leTorqCalc();

void JointControlUpdateStep(){
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
} 

void desiredTrajectory(double t){//PROSOXH!: allagh ton indexes apo matlab se c++ stous pinakes
    
    ke_star << 10000, 0, 0,
                0, 10000, 0,
                0, 0, 10000;
    
    kd_e << 100, 0, 0,
            0, 100, 0,
            0, 0, 100;

    kd_b << 100, 0, 0,
            0, 100, 0,
            0, 0, 100;

    md_e << kd_e(0,0)/(wn_free*wn_free), 0, 0,
            0, kd_e(1,1)/(wn_free*wn_free), 0,
            0, 0, kd_e(2,2)/(wn_free*wn_free);
    double v =  (fdes(0)*z_contact)/(md_e(0,0)*wn_contact); 
    double x_target_in = 10, y_target_in = 2.5, theta_target_in = 0;
    double xE_in = 6, yE_in = 7;
    a_matrix << 1, t0, pow(t0,2), pow(t0,3), pow(t0,4), pow(t0,5),
                0, 1, 2*t0, 3*pow(t0,2), 4*pow(t0,3), 5*pow(t0,4),
                0, 0, 2, 6*t0, 12*pow(t0,2), 20*pow(t0,3),
                1, t_free, pow(t_free,2), pow(t_free,3), pow(t_free,4), pow(t_free,5),
                0, 1, 2*t_free, 3*pow(t_free,2), 4*pow(t_free,3), 5*pow(t_free,4),
                0, 0, 2, 6*t_free, 12*pow(t_free,2), 20*pow(t_free,3) ;
    ///////initial sinthikes///////////////
    double sin_x = 0, sdotin_x = 0, sdotdotin_x = 0;
    double sin_y = 0, sdotin_y = 0, sdotdotin_y = 0;
    double sin_theta = 0, sdotin_theta = 0, sdotdotin_theta = 0;
    ///////telikes sinthikes/////////////
    double xE_contact = x_target_in - l0;
    double yE_contact = y_target_in;
    double thetaE_contact = theta_target_in;
    double sfin_x = 1, sdotfin_x = v/(xE_contact-xE_in), sdotdotfin_x = 0;
    double sfin_y = 1, sdotfin_y = 0, sdotdotfin_y = 0;
    double sfin_theta = 1, sdotfin_theta = 0, sdotdotfin_theta = 0;

    

    b1_x << sin_x, sdotin_x, sdotdotin_x, sfin_x, sdotfin_x, sdotdotfin_x;
    b1_y <<sin_y, sdotin_y, sdotdotin_y, sfin_y, sdotfin_y, sdotdotfin_y;
    b1_theta << sin_theta, sdotin_theta, sdotdotin_theta, sfin_theta, sdotfin_theta, sdotdotfin_theta;

    //kanonika edo thelei ena if t<=t_free (??)
    if(t<=t_free){
        a_x = a_matrix.inverse()*b1_x;
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
        xEd=xE_in+s_x*(xE_contact-xE_in);
        xEddot=sdot_x*(xE_contact-xE_in);
        xEddotdot=sdotdot_x*(xE_contact-xE_in);
        /////
        yEd=xE_in+s_y*(yE_contact-yE_in);
        yEddot=sdot_y*(yE_contact-yE_in);
        yEddotdot=sdotdot_y*(yE_contact-yE_in);
        /////
        thetaEd=thetaE_in+s_theta*(thetaE_contact-thetaE_in);
        thetaEddot=sdot_theta*(thetaE_contact-thetaE_in);
        thetaEddotdot=sdotdot_theta*(thetaE_contact-thetaE_in);
    }
    else{
        xEd=xt+xE_contact-xt_in; //anti gia x_target=xt
        xEddot=xtdot;
        xEddotdot=fext(0)/mt;
        /////
        yEd=yE_contact;
        yEddot=0;
        yEddotdot=0;
        /////
        thetaEd=thetaE_contact;
        thetaEddot=0;
        thetaEddotdot=0; 

    }
    xd(0) = xEd;
    xd(1) = yEd;
    xd(2) = thetaEd;
    xddot(0) = xEddot;
    xddot(1) = yEddot;
    xddot(2) = thetaEddotdot;
    xddotdot(0) = xEddotdot;
    xddotdot(1) = yEddotdot;
    xddotdot(2) = thetaEddotdot;
    


}

void trajparams(double t){
    double a1 = 10000;
    double a2 = 0.0001;
    s_x = a5x*pow(t,5) + a4x*pow(t,4) + a3x*pow(t,3) + a2x*pow(t,2) + a1x*t +a0x;
    s_y = a5y*pow(t,5) + a4y*pow(t,4) + a3y*pow(t,3) + a2y*pow(t,2) + a1y*t +a0y;
    s_theta = a5t*pow(t,5) + a4t*pow(t,4) + a3t*pow(t,3) + a2t*pow(t,2) + a1t*t +a0t;
    xdf(0) = xch_in + s_x*(xt_in-xch_in); 
    xdf(1) = ych_in + s_y*(yt_in-ych_in);
    xdf(2) = thetach_in + s_theta*(thetat_in-thetach_in);
    xdc(0) = xt;
    xdc(1) = yt;
    xdc(2) = thetat;
    xd = xdf*((abs(1-fext.squaredNorm()/a1))/(1+a1*fext.squaredNorm())) + xdc*fext.squaredNorm()/(fext.squaredNorm()+a2);
}

/////////////// CALCULATION FUNCTIONS DEFINITION END////////////////////////


