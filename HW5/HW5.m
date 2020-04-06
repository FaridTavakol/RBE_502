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
%% GENERATE TRAJECTORY Problem 2
% initial state
t0=0;
q0=[-0.5,0.2,0.1,0.1];  % initial condition % feel free to change
% final state
tf=10; % time span is 10 seconds
% qf=[pi/3*180/pi pi/4*180/pi 0 0];
qf=[0 0 0 0];
% generating the desired trajectory based on the initial and terminal
% state conditions
link1= TajectoryGenerator( q0(1), q0(3), qf(1), qf(3),t0,tf);
link2= TajectoryGenerator( q0(2), q0(4), qf(2), qf(4),t0,tf);


%% Implement the PD+ GRAVITY COMPENSATION control for set point tracking.
integral_term = false ;
xf = [0, 0, 0, 0];
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) PDControlGravity(t,x,link1,link2,integral_term),[t0 tf],q0, options);


figure('Name','Theta_1 under PD SetPoint Control');
plot(T, X(:,1),'r-');
hold on

figure('Name','Theta_2 under PD SetPoint Control');
plot(T, X(:,2),'r--');
hold on

%% TODO: GENERATE TRAJECTORY USING TwoLinkArmTraj matlab file.



