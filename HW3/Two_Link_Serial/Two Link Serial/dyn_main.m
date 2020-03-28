%%%%%% Robotics HW4 (Kinematic and Dynamic control and analysis for 2-link serial chain manipulator)

% Course: Robotic Manipulation and Mobility
% Advisor: Dr. V. Krovi
%
% Homework Number: 4
%
% Names: Sourish Chakravarty
% 	Hrishi Lalit Shah


clc
clear all
close all

%% INITIALIZATION
global rx ry ell_an start_an w Kp Kv iterations thdotdot thdot theta
global error1 error2 error3 error4 error5 error6 error7
global l1 l2 lc1 lc2 j1 j2 m1 m2 g A B rx ry ell_an w Kp Kx Kp1 Kd1 % Given parameters

%% INPUT PARAMETERS
l1= 2 ;% (m)Length of element 1
lc1= 1 ;%(m) Distance of CM of element 1 from source end
m1= 10;% (kg) Mass of element 1
j1= 3;% (kg-m^2) Moment of Inertia of element 1 about its CM
tau1= 0;% (N-m) External torque applied on element 1

l2= 1;% (m)Length of element 2
lc2= 0.5;% (m)Distance of CM of element 2 from source end
m2= 6;% (kg) Mass of element 2
j2= 2;% (kg-m^2)Moment of Inertia of element 2 about its CM
tau2= 0;% (N-m)External torque applied on element 2

g= 9.81; % (m/sec^2) Acceleration due to gravity

%%

%% DESIRED TRAJECTORY DATA
d2r=pi/180; %degrees to radians
w=72*d2r; % rotational velocity rad/s
rx=1.75; ry=1.25; % ellipse radii
ell_an=45*d2r; % angle of inclination of ellipse
%start_an=45*d2r; % angle of ellipse
A=0;
B=0;

%% INITIAL CONDITIONS
x_0=1.75*cos(45*d2r)+0.1 % initial x in task space
y_0=1.25*sin(45*d2r)+0.1 % initial y in task space

xd_0= -rx*w*sin(w*0)*cos(ell_an)-ry*w*cos(w*0)*sin(ell_an); % initial x-velcity in task space
yd_0= -rx*w*sin(w*0)*sin(ell_an)+ry*w*cos(w*0)*cos(ell_an);% initial y-velcity in task space
% th_0=invbot_new([x_0, y_0]); % initial position in joint space
th_0=invbot([x_0, y_0]); % initial position in joint space
J=[-l1*sin(th_0(1)),-l2*sin(th_0(2));
    l1*cos(th_0(1)), l2*cos(th_0(2))];
thd_0=(inv(J)*[xd_0,yd_0]')'; % initial joint space velocities

%% SIMULATION PARAMETERS
n=360; % total number of points to be simulated
sim_time=10; % 20 second simulation
dt=sim_time/n; % each time step in a 20 second simulation
tspan=0:dt:sim_time;
options = odeset('RelTol',1e-4);

%     case 1 %% Kinematic Simulation: Open Loop
%     case 2    %% Kinematic Simulation: Closed Loop-Joint Space Kp=1,1
%     case 3    %% Kinematic Simulation: Closed Loop-Joint Space Kp=100,100
%     case 4    %% Kinematic Simulation: Closed Loop-Joint Space Kp=100,1
%     case 5 %% Kinematic Simulation: Closed Loop-Task Space Kx=1,1
%     case 6 %% Kinematic Simulation: Closed Loop-Task Space Kx= 100,100
%     case 7 %% Kinematic Simulation: Closed Loop-Task Space Kx= 100,10
%     case 8    %% Dynamic Simulation: Open Loop
%     case 9 %% Dynamic Closed Loop (B1)
%     case 10 %% Dynamic Closed Loop (B2)
%     case 11 %% Dynamic Closed Loop (B3)

TRIG= 11;
switch (TRIG)
    case 1 %% Kinematic Simulation: Open Loop
        global itr Q1 t1
        itr=0;
        txt1=['KIN_OPEN LOOP'];
        [t,Y1]=ode45(@KIN_OL,tspan,[th_0],options);
        plotbot_js_new(tspan,Y1,1,txt1);
        ERROR_PLOT_1(t,Y1,1,txt1);

        %%%%% To find torque from simulated Theta
        tau_sim = FIND_TAU_SIM(tspan,Y1)
        figure()
        plot(tspan,tau_sim);
        ylabel('Torque');
        xlabel('Time(sec)');
        title('Torque from simulated data');

    case 2    %% Kinematic Simulation: Closed Loop-Joint Space Kp=1,1
        global itr Q2 t2
        itr=0;
        Kp= [1 0;0 1];
        txt1=['KIN-CLOSED LOOP-JS (Kp=',num2str(Kp(1,1)),',',num2str(Kp(2,2)),')'];
        [t,Y2]=ode45(@KIN_CLJS,tspan,[th_0],options);
        plotbot_js_new(tspan,Y2,2,txt1);
        ERROR_PLOT_1(t,Y2,2,txt1);

    case 3    %% Kinematic Simulation: Closed Loop-Joint Space Kp=100,100
        global itr Q2 t2
        itr=0;
        Kp= [100 0;0 100];
        txt1=['KIN-CLOSED LOOP-JS (Kp=',num2str(Kp(1,1)),',',num2str(Kp(2,2)),')'];
        [t,Y2]=ode45(@KIN_CLJS,tspan,[th_0],options);
        plotbot_js_new(tspan,Y2,2,txt1);
        ERROR_PLOT_1(t,Y2,2,txt1);

    case 4    %% Kinematic Simulation: Closed Loop-Joint Space Kp=100,1
        global itr Q2 t2
        itr=0;
        Kp= [100 0;0 1];
        txt1=['KIN-CLOSED LOOP-JS (Kp=',num2str(Kp(1,1)),',',num2str(Kp(2,2)),')'];
        [t,Y2]=ode45(@KIN_CLJS,tspan,[th_0],options);
        plotbot_js_new(tspan,Y2,2,txt1);
        ERROR_PLOT_1(t,Y2,2,txt1);

    case 5 %% Kinematic Simulation: Closed Loop-Task Space Kx=1,1
        global itr Q3 t3
        itr=0;
        Kx= [1 0;0 1];
        txt1=['KIN-CLOSED LOOP-TS (Kx=',num2str(Kx(1,1)),',',num2str(Kx(2,2)),')'];
        [t,Y3]=ode45(@KIN_CLTS,tspan,[th_0],options);
        plotbot_js_new(tspan,Y3,3,txt1);
        ERROR_PLOT_1(t,Y3,3,txt1);

    case 6 %% Kinematic Simulation: Closed Loop-Task Space Kx= 100,100
        global itr Q3 t3
        itr=0;
        Kx= [100 0;0 100];
        txt1=['KIN-CLOSED LOOP-TS (Kx=',num2str(Kx(1,1)),',',num2str(Kx(2,2)),')'];
        [t,Y3]=ode45(@KIN_CLTS,tspan,[th_0],options);
        plotbot_js_new(tspan,Y3,3,txt1);
        ERROR_PLOT_1(t,Y3,3,txt1);

    case 7 %% Kinematic Simulation: Closed Loop-Task Space Kx= 100,10
        global itr Q3 t3
        itr=0;
        Kx= [100 0;0 10];
        txt1=['KIN-CLOSED LOOP-TS (Kx=',num2str(Kx(1,1)),',',num2str(Kx(2,2)),')'];
        [t,Y3]=ode45(@KIN_CLTS,tspan,[th_0],options);
        plotbot_js_new(tspan,Y3,3,txt1);
        ERROR_PLOT_1(t,Y3,3,txt1);
        
%% DYNAMIC SIMULATION
    case 8    %% Dynamic Simulation: Open Loop
        global itr Q4 t4 Tau1
        itr=0;
        txt1=['DYN-OPEN LOOP'];
        [t,Y4]=ode45(@DYN_OL,tspan,[th_0,thd_0],options);
        plotbot_js_new(t,Y4,4,txt1);
        ERROR_PLOT_2(t,Y4,4,txt1);
        figure
        plot(t4,Tau1);
        xlabel('Time (sec)');
        ylabel('Torque');
        title(txt1);
        
    case 9 %% Dynamic Closed Loop (B1)
        global itr Q5 t5 Tau2
        itr=0;
        Kp1= [1000, 0;0, 1000];%%%Dynamic analysis
        Kd1= [10, 0;0, 10];%%%Dynamic analysis
        txt1=['DYN-CLOSED LOOP-B1 (Kp=',num2str(Kp1(1)),',Kd=',num2str(Kd1(1)),')'];
        [t,Y5]=ode45(@DYN_CL_B1,tspan,[th_0,thd_0],options);
        plotbot_js_new(tspan,Y5,5,txt1);
        ERROR_PLOT_2(t,Y5,5,txt1);
        figure
        plot(t5,Tau2);
        xlabel('Time (sec)');
        ylabel('Torque');
        title(txt1);
    case 10 %% Dynamic Closed Loop (B2)
        global itr Q6 t6 Tau3
        itr=0;
        
        Kp1= [1000, 0;0, 1000];%%%Dynamic analysis
        Kd1= [100, 0;0, 100];%%%Dynamic analysis
        txt1 = ['DYN-CLOSED LOOP-B2 (Kp=',num2str(Kp1(1)),',Kd=',num2str(Kd1(1)),')'];
        [t,Y6]=ode45(@DYN_CL_B2,tspan,[th_0,thd_0],options);
        plotbot_js_new(tspan,Y6,6,txt1);
        ERROR_PLOT_2(t,Y6,6,txt1);
        figure
        plot(t6,Tau3);
        xlabel('Time (sec)');
        ylabel('Torque');
        title(txt1);
    case 11 %% Dynamic Closed Loop (B3)
        global itr Q7 t7 Tau4
        itr=0;
        Kp1= [100, 0;0, 100];%%%Dynamic analysis
        Kd1= [20, 0;0, 20];%%%Dynamic analysis
        txt1=['DYN-CLOSED LOOP-B3 (Kp=',num2str(Kp1(1)),',Kd=',num2str(Kd1(1)),')'];
        [t,Y7]=ode45(@DYN_CL_B3,tspan,[th_0,thd_0],options);
        plotbot_js_new(tspan,Y7,7,txt1);
        ERROR_PLOT_2(t,Y7,7,txt1);
        figure
        plot(t7,Tau4);
        xlabel('Time (sec)');
        ylabel('Torque');
        title(txt1);
    otherwise
        'ENTER CORRECT TRIGGER VALUE'
end