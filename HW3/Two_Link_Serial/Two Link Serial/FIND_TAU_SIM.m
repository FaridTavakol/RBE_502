%%% CALCULATES TORQUE BASED ON JOINT SPACE INFORMATION FROM KINEMATIC
%%% SIMULATION

% Course: Robotic Manipulation and Mobility
% Advisor: Dr. V. Krovi
%
% Homework Number: 4
%
% Names: Sourish Chakravarty
% 	Hrishi Lalit Shah

function TAU_SIM=FIND_TAU_SIM(tspan,X)

global l1 l2 lc1 lc2 j1 j2 m1 m2 g rx ry ell_an w Kp Kx Kp1 Kd1 A B % Given parameters
global itr Q1 Q2 Q3 Q4 Q5 Q6 Q7 Q8 t1 t2 t3 t4 t5 t6 t7 t8

for i=1:length(tspan)
    t=tspan(i);
    th1=X(i,1);
    th2=X(i,2);
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
    th_des=invbot2([x_des, y_des],[th1, th2]); % position in joint space

    %J=[-l1*sin(th_des(1)) -l2*sin(th_des(2));
    %  l1*cos(th_des(1))  l2*cos(th_des(2))];
    %% Simulation update
    J=[-l1*sin(th1) -l2*sin(th2);
        l1*cos(th1)  l2*cos(th2)];

    THD=inv(J)*[xd_des,yd_des]';
    dX=[THD];

    %% Determining angular accelerations
    th1d=THD(1);
    th2d=THD(2);

    Jdot= [-l1*cos(th1)*th1d, -l2*cos(th2)*th2d;
        -l1*sin(th1)*th1d, -l2*sin(th2)*th2d;];
%     
%      inv(J)
%      ([xdd_des;ydd_des])
%       - Jdot*THD
    THDD= inv(J)*([xdd_des;ydd_des] - Jdot*THD);


    %%%%%%%%%%%%%%%%%%%%%%% CREATING IMPORTANT MATRICES IN THE GOVERNING EQUATION
    s1=sin(th1);
    s2=sin(th2);
    c1=cos(th1);
    c2=cos(th2);
    c21=cos(th2-th1);
    s21=sin(th2-th1);

    % Kp=[1 0;
    %     0 1];
    %%%%%% Dynamic control matrices
    M = [ j1+m2*l1^2, m2*l1*lc2*c21;
        m2*l1*lc2*c21, j2+m2*lc2^2];
    V = [0, -m2*l1*lc2*s21;
        m2*l1*lc2*s21, 0];
    G = [m1*g*lc1*c1+m2*g*l1*c1;
        m2*g*lc2*c2];
    %%%%%%%%%%%%%%%%%%%%%%%%%
    TAU_SIM(i,:)= [M*[THDD(1);THDD(2)] + V*[th1d^2;th2d^2]+G]';
end

