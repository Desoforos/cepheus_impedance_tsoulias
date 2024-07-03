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
  2*i0zz+2*i1zz+2*i2zz+2*i3zz+m1*(2*pow((r0x*cos(theta0)+l1*cos(q1+theta0) 
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