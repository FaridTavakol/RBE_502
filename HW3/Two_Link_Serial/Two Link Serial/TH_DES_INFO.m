%%%%%%%% EVALUATES DESIRED THETA, THETA_DOT, THETA_DOTDOT AT ANY GIVEN TIME 

% Course: Robotic Manipulation and Mobility
% Advisor: Dr. V. Krovi
%
% Homework Number: 4
%
% Names: Sourish Chakravarty
% 	Hrishi Lalit Shah

function [THETA_DES]=TH_DES_INFO(t,X)

global l1 l2 lc1 lc2 j1 j2 m1 m2 g rx ry ell_an w Kp Kx Kp1 Kd1 A B % Given parameters


th1=X(1); %%%Initial approximation for theta 1 
th2=X(2);


%% Trajectory information
R1=rx;
R2=ry;
r=ell_an;

x_des= A + R1*cos(w*t)*cos(r)-R2*sin(w*t)*sin(r);
y_des= B + R1*cos(w*t)*sin(r)+R2*sin(w*t)*cos(r);

xd_des= -R1*w*sin(w*t)*cos(r)-R2*w*cos(w*t)*sin(r);
yd_des= -R1*w*sin(w*t)*sin(r)+R2*w*cos(w*t)*cos(r);

xdd_des = -R1*(w^2)*cos(w*t)*cos(r)+R2*(w^2)*sin(w*t)*sin(r);
ydd_des = -R1*(w^2)*cos(w*t)*sin(r)-R2*(w^2)*sin(w*t)*cos(r);
% th_des=invbot_new([x_des, y_des]); % position in joint space
% th_des=invbot([x_des, y_des]); % position in joint space

th_des=invbot2([x_des, y_des],[th1, th2]); % position in joint space (using simulation data for a better convergence)


J=[-l1*sin(th_des(1)) -l2*sin(th_des(2));
   l1*cos(th_des(1))  l2*cos(th_des(2))];

% J=[-l1*sin(th1) -l2*sin(th2);
%     l1*cos(th1)  l2*cos(th2)];
% x1= l1*cos(th1) + l2*cos(th2);
% y1= l1*sin(th1) + l2*sin(th2);

THD=inv(J)*([xd_des,yd_des]'); % Desired angular velocity


%% Determining angular accelerations
th1d=THD(1);
th2d=THD(2);

% Jdot= [-l1*cos(th1)*th1d, -l2*cos(th2)*th2d;
%         -l1*sin(th1)*th1d, -l2*sin(th2)*th2d;];
Jdot= [-l1*cos(th_des(1))*th1d, -l2*cos(th_des(2))*th2d;
        -l1*sin(th_des(1))*th1d, -l2*sin(th_des(2))*th2d;];    
    
THDD= inv(J)*([xdd_des;ydd_des] - Jdot*THD);    % Desired angular accelerations

% THETA_DES=[th1,th2,THD',THDD'];
THETA_DES=[th_des(1),th_des(2),THD',THDD'];

