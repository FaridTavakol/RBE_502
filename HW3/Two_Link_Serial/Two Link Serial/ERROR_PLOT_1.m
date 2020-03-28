%%%% ERROR CALCULATION FOR KINEMATIC ANALYSIS

% Course: Robotic Manipulation and Mobility
% Advisor: Dr. V. Krovi
%
% Homework Number: 4
%
% Names: Sourish Chakravarty
% 	Hrishi Lalit Shah


function []= ERROR_PLOT_1(tspan,X,h,txt1)


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

    xa= l1*cos(th1) + l2*cos(th2); %%% Actual end effector positions (TS)
    ya= l1*sin(th1) + l2*sin(th2);
    x_err(i,:)= [x_des - xa, y_des - ya, sqrt((x_des - xa)^2+(y_des - ya)^2)]; %%% Task Space Error
    th_err(i,:)= [th_des(1) - th1, th_des(2) - th2, sqrt((th_des(1) - th1)^2+(th_des(2) - th2)^2)];%%% Joint Space Error
end
% Error plot
figure(3*h-1)
subplot(3,1,1)
plot(tspan,x_err(:,1));
ylabel('xd-x');
title([txt1,':End Effector position error plot']);
subplot(3,1,2)
plot(tspan,x_err(:,2));
ylabel('yd-y')
subplot(3,1,3)
plot(tspan,x_err(:,3));
ylabel('norm(Xd-X)')
xlabel('Time (sec)');

figure(3*h)
subplot(3,1,1)
plot(tspan,th_err(:,1));
ylabel('th1d-th1');
title([txt1,':Joint Space error plots']);
subplot(3,1,2)
plot(tspan,th_err(:,2));
ylabel('th2d-th2')
subplot(3,1,3)
plot(tspan,th_err(:,3));
ylabel('norm(THd-TH)')
xlabel('Time (sec)');

