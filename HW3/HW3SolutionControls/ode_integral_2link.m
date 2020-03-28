function dedt = ode_integral_2link(t,e,params)
%Params is a 8x1 vector with the constants of the cubic polynomial
%trajectories of first and second joints.
A = [0 1 0; 0 0 1; 0 0 0];
B = [0; 0; 1];
K = place(A,B,[-3 -5 -7]);
%
dedt = zeros(6,1);
e = num2cell(e);
[e1_i, e2_i, e1, e2, e1_dot, e2_dot] = deal(e{:});
if abs(e1) > 2*pi
e1 = mod(e1, 2*pi);
end
if abs(e2) > 2*pi
e2 = mod(e2, 2*pi);
end
%defining desired trajectories at time t
d_q1 = params(1) + (params(2)*t) + (params(3)*t^2) + (params(4)*t^3);
d_q2 = params(5) + (params(6)*t) + (params(7)*t^2) + (params(8)*t^3);
d_q1_dot = (params(2)) + (2*params(3)*t) + (3*params(4)*t^2); %first time derivative
d_q2_dot = (params(6)) + (2*params(7)*t) + (3*params(8)*t^2); %first time derivative
d_acc1 = (2*params(3)) + (6*params(4)*t); %second time derivative
d_acc2 = (2*params(7)) + (6*params(8)*t); %second time derivative
%control input
acc1 = (-K*[e1_i; e1; e1_dot]) + d_acc1;
acc2 = (-K*[e2_i; e2; e2_dot]) + d_acc2;
acc = [acc1;acc2];
%2-link Arm parameters, (given)
I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
g=9.8;
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;
%
M = [a+2*b*cos(e{4}+d_q2), d+b*cos(e{4}+d_q2);
    d+b*cos(e{4}+d_q2), d];
C = [-b*sin(e{4}+d_q2)*(e{6}+d_q2_dot), -b*sin(e{4}+d_q2)*(e{5}+d_q1_dot+e{6}+d_q2_dot); b*sin(e{4}+d_q2)*(e{5}+d_q1_dot),0];
G = [m1*g*r1*cos(e{3}+d_q1)+m2*g*(l1*cos(e{3}+d_q1)+r2*cos(e{3}+d_q1+e{4}+d_q2));
    m2*g*r2*cos(e{3}+d_q1+e{4}+d_q2)];
%
invM = inv(M);
q_dot_vec = [e{5}+d_q1_dot; e{6}+d_q2_dot];
%Calculating torque
Tau = (M*acc) + (C*q_dot_vec) + G;
% Expression for qi_ddot 
qi_ddot = invM*(Tau - (C*q_dot_vec) - G);
%Defining the dynamics with first order ODEs
dedt(1) = e{3};
dedt(2) = e{4};
dedt(3) = e{5};
dedt(4) = e{6};
dedt(5) = qi_ddot(1) - d_acc1;
dedt(6) = qi_ddot(2) - d_acc2;
end