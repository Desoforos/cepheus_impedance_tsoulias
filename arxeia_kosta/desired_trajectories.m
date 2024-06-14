function W=desired_trajectories(t,target_pose,target_velocity,Fext,parameters)
% 
t0=parameters(23);
t_free=parameters(24);
v=parameters(41);
xE_in=parameters(25);
yE_in=parameters(26);
thetaE_in=parameters(27);
xE_fin=parameters(34);
yE_fin=parameters(35);
thetaE_fin=parameters(36);
S01=parameters(21);
S02=parameters(22);
mt=parameters(5);
It=parameters(11);
l0=parameters(44);
%
x_target=target_pose(1);
y_target=target_pose(2);
theta_target=target_pose(3);
%
xdot_target=target_velocity(1);
ydot_target=target_velocity(2);
thetadot_target=target_velocity(3);
% t=u(1);
% xdot_target=u(2);
% ydot_target=u(3);
% thetadot_target=u(4);
% x_target=u(5);
% y_target=u(6);
% theta_target=u(7);
Fext_x=Fext(1);
Fext_y=Fext(2);
next=Fext(3);
%
% t=105;
% xdot_target=0.2;
% ydot_target=0.3;
% thetadot_target=0.8;
% x_target=0.4;
% y_target=0.5;
% theta_target=0.9;
% Fext_x=0.6;
% Fext_y=0.7;
% next=0.8;
%
% %
% Chaser End-Effector
%
A=[1 t0 t0^2 t0^3 t0^4 t0^5;
    0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
    0 0 2 6*t0 12*t0^2 20*t0^3;
    1 t_free t_free^2 t_free^3 t_free^4 t_free^5;
    0 1 2*t_free 3*t_free^2 4*t_free^3 5*t_free^4;
    0 0 2 6*t_free 12*t_free^2 20*t_free^3];
% 
sin_x=0;
sdotin_x=0;
sdotdotin_x=0;
%  
sin_y=0;
sdotin_y=0;
sdotdotin_y=0;
% 
sin_theta=0;
sdotin_theta=0;
sdotdotin_theta=0;
% 
sfin_x=1;
sdotfin_x=v/(xE_fin-xE_in);
sdotdotfin_x=0;
% 
sfin_y=1;
sdotfin_y=0;
sdotdotfin_y=0;
% 
sfin_theta=1;
sdotfin_theta=0;
sdotdotfin_theta=0;
% 
B1_x=[sin_x;sdotin_x;sdotdotin_x;sfin_x;sdotfin_x;sdotdotfin_x];
% 
B1_y=[sin_y;sdotin_y;sdotdotin_y;sfin_y;sdotfin_y;sdotdotfin_y];
% 
B1_theta=[sin_theta;sdotin_theta;sdotdotin_theta;sfin_theta;sdotfin_theta;sdotdotfin_theta];
% 
if t<=t_free 
a_x=A\B1_x;
a0_x=a_x(1,1);
a1_x=a_x(2,1);
a2_x=a_x(3,1);
a3_x=a_x(4,1);
a4_x=a_x(5,1);
a5_x=a_x(6,1);
s_x=a0_x+a1_x*t+a2_x*t.^2+a3_x*t.^3+a4_x*t.^4+a5_x*t.^5;
sdot_x=a1_x+2*a2_x*t+3*a3_x*t.^2+4*a4_x*t.^3+5*a5_x*t.^4;
sdotdot_x=2*a2_x+6*a3_x*t+12*a4_x*t.^2+20*a5_x*t.^3;
% 
xE_d=xE_in+s_x*(xE_fin-xE_in);
xEdot_d=sdot_x*(xE_fin-xE_in);
xEdotdot_d=sdotdot_x*(xE_fin-xE_in);
%  
a_y=A\B1_y;
a0_y=a_y(1,1);
a1_y=a_y(2,1);
a2_y=a_y(3,1);
a3_y=a_y(4,1);
a4_y=a_y(5,1);
a5_y=a_y(6,1);
s_y=a0_y+a1_y*t+a2_y*t.^2+a3_y*t.^3+a4_y*t.^4+a5_y*t.^5;
sdot_y=a1_y+2*a2_y*t+3*a3_y*t.^2+4*a4_y*t.^3+5*a5_y*t.^4;
sdotdot_y=2*a2_y+6*a3_y*t+12*a4_y*t.^2+20*a5_y*t.^3;
% 
yE_d=yE_in+s_y*(yE_fin-yE_in);
yEdot_d=sdot_y*(yE_fin-yE_in);
yEdotdot_d=sdotdot_y*(yE_fin-yE_in);
%  
a_theta=A\B1_theta;
a0_theta=a_theta(1,1);
a1_theta=a_theta(2,1);
a2_theta=a_theta(3,1);
a3_theta=a_theta(4,1);
a4_theta=a_theta(5,1);
a5_theta=a_theta(6,1);
%
s_theta=a0_theta+a1_theta*t+a2_theta*t.^2+a3_theta*t.^3+a4_theta*t.^4+a5_theta*t.^5;
sdot_theta=a1_theta+2*a2_theta*t+3*a3_theta*t.^2+4*a4_theta*t.^3+5*a5_theta*t.^4;
sdotdot_theta=2*a2_theta+6*a3_theta*t+12*a4_theta*t.^2+20*a5_theta*t.^3;
% 
thetaE_d=thetaE_in+s_theta*(thetaE_fin-thetaE_in);
thetaEdot_d=sdot_theta*(thetaE_fin-thetaE_in);
thetaEdotdot_d=sdotdot_theta*(thetaE_fin-thetaE_in);
%
% Chaser Base
% 
xb_d=xE_d-S01;
xbdot_d=xEdot_d;
xbdotdot_d=xEdotdot_d;
% 
yb_d=yE_d-S02;
ybdot_d=yEdot_d;
ybdotdot_d=yEdotdot_d;
% 
thetab_d=thetaE_d;
thetabdot_d=thetaEdot_d;
thetabdotdot_d=thetaEdotdot_d;
else
% 
xdotdot_target=Fext_x/mt;
%
xE_d=x_target-0.015;
xEdot_d=xdot_target;
xEdotdot_d=xdotdot_target;
%
% 
ydotdot_target=Fext_y/mt;
%
yE_d=y_target;
yEdot_d=ydot_target;
yEdotdot_d=ydotdot_target;
%
% 
thetadotdot_target=next/It;
%
thetaE_d=theta_target;
thetaEdot_d=thetadot_target;
thetaEdotdot_d=thetadotdot_target;
%
xb_d=xE_d-S01;
xbdot_d=xEdot_d;
xbdotdot_d=xEdotdot_d;
% 
yb_d=yE_d-S02;
ybdot_d=yEdot_d;
ybdotdot_d=yEdotdot_d;
% 
thetab_d=thetaE_d;
thetabdot_d=thetaEdot_d;
thetabdotdot_d=thetaEdotdot_d;
%
end
%
%
rE_d=[xE_d;yE_d];
rEdot_d=[xEdot_d;yEdot_d];
rEdotdot_d=[xEdotdot_d;yEdotdot_d];
%
rb_d=[xb_d;yb_d];
rbdot_d=[xbdot_d;ybdot_d];
rbdotdot_d=[xbdotdot_d;ybdotdot_d];
%
w=[rE_d;thetaE_d;rb_d;thetab_d];
%
wdot=[rEdot_d;thetaEdot_d;rbdot_d;thetabdot_d];
%
wdotdot=[rEdotdot_d;thetaEdotdot_d;rbdotdot_d;thetabdotdot_d];
%
W=[w;wdot;wdotdot];