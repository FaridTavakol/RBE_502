% Notations: For a given variable, x, dx is its time derivative, ddx is
% 2nd-order derivative. 
clc
clear all;
close all;
% the following parameters for the arm
I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
g=9.8;

% we compute the parameters in the dynamic model
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;

%% Example to show computing the symbolic expression of M and C. But not needed later as subs() function in matlab extremely slow.
% create symbolic variable for x.
% x1 - theta1
% x2 - theta2

symx= sym('symx',[4,1]); 

M = [a+2*b*cos(symx(2)), d+b*cos(symx(2));
    d+b*cos(symx(2)), d];
C = [-b*sin(symx(2))*symx(4), -b*sin(symx(2))*(symx(3)+symx(4)); b*sin(symx(2))*symx(3),0];
G = [m1*g*r1*cos(symx(1))+m2*g*(l1*cos(symx(1))+r2*cos(symx(1)+symx(2)));
    m2*g*r2*cos(symx(1)+symx(2))];


invM = inv(M);
invMC= inv(M)*C;

% the options for ode
% initial condition % feel free to change
x0= [-0.5,0.2,0.1,0.1];



%% Implement the PD+ GRAVITY COMPENSATION control for set point tracking.
xf = [0, 0, 0, 0];
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) PDControlGravity(t,x),[0 tf],x0, options);

% 
% figure('Name','Theta_1 under PD SetPoint Control');
% plot(T, X(:,1),'r-');
% hold on
% 
% figure('Name','Theta_2 under PD SetPoint Control');
% plot(T, X(:,2),'r--');
% hold on


%% Implement the iterative learning control (assume no knowledge about the dynamic model).
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) ILCtrl(t,x),[0 tf],x0, options);

% figure('Name','Theta_1 under iterative learning control');
% plot(T, X(:,1),'r-');
% hold on
% figure('Name','Theta_2 under iterative learning control');
% plot(T, X(:,2),'r--');
% hold on


%% TODO: GENERATE TRAJECTORY USING TwoLinkArmTraj matlab file.
%% Implement the inverse dynamic control  
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) inverseDC(t,x),[0 tf],x0, options);

% figure('Name','Theta_1 under inverse dynamic control');
% plot(T, X(:,1),'r-');
% hold on
% figure('Name','Theta_2 under inverse dynamic control');
% plot(T, X(:,2),'r--');
% hold on

%% Implement the lyapunov-based control  
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) lyapunovCtrl(t,x),[0 tf],x0, options);

% figure('Name','Theta_1 under lyapunov-based control');
% plot(T, X(:,1),'r-');
% hold on
% figure('Name','Theta_2 under  lyapunov-basedcontrol');
% plot(T, X(:,2),'r--');
% hold on



%% Implement the passivity-based control  
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) passivityCtrl(t,x),[0 tf],x0, options);

% figure('Name','Theta_1 under lyapunov-based control');
% plot(T, X(:,1),'r-');
% hold on
% figure('Name','Theta_2 under  lyapunov-basedcontrol');
% plot(T, X(:,2),'r--');
% hold on