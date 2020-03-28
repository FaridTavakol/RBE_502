%%%% DYNAMIC ANALYSIS : Open Loop 

% Course: Robotic Manipulation and Mobility
% Advisor: Dr. V. Krovi
%
% Homework Number: 4
%
% Names: Sourish Chakravarty
% 	Hrishi Lalit Shah

function [dX]= DYN_OL(t,X)


global l1 l2 lc1 lc2 j1 j2 m1 m2 g rx ry ell_an w Kp Kx Kp1 Kd1 A B % Given parameters
global itr Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8 t1 t2 t3 t4 t5 t6 t7 t8
global Tau1 Tau2 Tau3 Tau4

th1=X(1);
th2=X(2);
th1d=X(3);
th2d=X(4);


%% CREATING IMPORTANT MATRICES IN THE GOVERNING EQUATION

%% Acquiring desired theta values for points exactly on the ellipse%%%%%%%
THETA_DES=TH_DES_INFO(t,[th1,th2]);
th1_des=THETA_DES(1);
th2_des=THETA_DES(2);
th1d_des=THETA_DES(3);
th2d_des=THETA_DES(4);
th1dd_des=THETA_DES(5);
th2dd_des=THETA_DES(6);

s1_des=sin(th1_des);
s2_des=sin(th2_des);
c1_des=cos(th1_des);
c2_des=cos(th2_des);
c21_des=cos(th2_des-th1_des);
s21_des=sin(th2_des-th1_des);

%% Dynamic control matrices : Desired
Mdes = [ j1+m2*l1^2, m2*l1*lc2*c21_des;
      m2*l1*lc2*c21_des, j2+m2*lc2^2];
Vdes = [0, -m2*l1*lc2*s21_des;
    m2*l1*lc2*s21_des, 0];
Gdes = [m1*g*lc1*c1_des+m2*g*l1*c1_des;
    m2*g*lc2*c2_des];

%% Control Torque (based on ideal situation)
Tau_des= Mdes*[th1dd_des;th2dd_des] + Vdes*[th1d_des^2;th2d_des^2] + Gdes; %% Control Torque

%% Simulation Data
s1=sin(th1);
s2=sin(th2);
c1=cos(th1);
c2=cos(th2);
c21=cos(th2-th1);
s21=sin(th2-th1);

%%%%%% Dynamic control matrices
M = [ j1+m2*l1^2, m2*l1*lc2*c21;
      m2*l1*lc2*c21, j2+m2*lc2^2];
V = [0, -m2*l1*lc2*s21;
    m2*l1*lc2*s21, 0];
G = [m1*g*lc1*c1+m2*g*l1*c1;
    m2*g*lc2*c2];
%%%%%%%%%%%%%%%%%%%%%%%%%

%% Generating differential
% Tau_des = M*[th1dd_des;th2dd_des] + V*[th1d_des^2;th2d_des^2] + Gdes; %% Control Torque
[THDD]=inv(M)*(Tau_des- V*[th1d^2;th2d^2]-G); %%% Deriving angular acc
% [THDD]=inv(M-Mdes)*(Vdes-V*[th1d^2;th2d^2] + (Gdes-G)); %%% Deriving angular acc
%% Simulation Update
dX(1,1)= th1d;
dX(2,1)= th2d;
dX(3,1)= THDD(1);
dX(4,1)= THDD(2);
%% Storage
itr=itr+1;
t4(itr)=t;
size(th1);
size(th2);
Q4(itr,1) = [th1];
Q4(itr,2) = [th2];
Q4(itr,3) = [th1d];
Q4(itr,4) = [th2d];
Q4(itr,5) = THDD(1);
Q4(itr,6) = THDD(2);
Tau1(itr,1:2) = Tau_des;
