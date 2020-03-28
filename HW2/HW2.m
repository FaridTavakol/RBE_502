%% HW2 Farid Tavakkolmoghaddam
clc; clear;
A=[0 1;-0.4 -0.2]; %System dynamics
B=[0;0.2];
C=[1 0];
D=0;
x0 = [0 0]; % initial position and velocity at the equilibrium point
sys_open_loop=ss(A,B,C,D) % open_loop transfer function
% desired poles at the  -1 -2.5 finding the gain (K)
P=[-1 -2.5];% desired poles 
disp('desired K that places the poles at the -1 , -2.5 :')
K=place(A,B,P)%desired gain
disp('desired Kr that places the poles at the -1 , -2.5 :')
Kr=-inv(C*inv((A-B*K))*B)% desired Kr
sys_closed_loop=ss(A-B*K,B*Kr,C,D) % closed loop system


t = 0:0.01:10;
u = 5*ones(size(t)); % input with the reference input of 5
% plotting the trajectory of the system
[y,t,x] = lsim(sys_closed_loop,u,t,x0);
figure('Name','With the equilibrium initial condition')
plot(t,y)
title('State trajectories (m) Vs. Time (sec)')
xlabel('Time (sec)')
ylabel('Position (m)')
grid 
%% Trying with the different initial position and velocity 
x0 = [5 2]; % initial position is 5 m and the initial velocit is 
% plotting the trajectory of the system
[y,t,x] = lsim(sys_closed_loop,u,t,x0);
figure('Name','With different initial condition')
plot(t,y)
title('State trajectories (m) Vs. Time (sec)')
xlabel('Time (sec)')
ylabel('Position (m)')
grid

