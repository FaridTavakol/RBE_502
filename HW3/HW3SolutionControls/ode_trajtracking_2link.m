function dzdt = ode_trajtracking_2link(t,z,params)
%Params is a 8x1 vector with the constants of the cubic polynomial
%trajectories of first and second joints.
A = [0 1; 0 0];
B = [0; 1];
K = place(A,B,[-3 -5]);
%
dzdt = zeros(4,1);
z = num2cell(z);
[q1, q2, q1_dot, q2_dot] = deal(z{:});
if abs(q1) > 2*pi
q1 = mod(q1, 2*pi);
end
if abs(q2) > 2*pi
q2 = mod(q2, 2*pi);
end
%defining desired trajectories
d_q1 = params(1) + (params(2)*t) + (params(3)*t^2) + (params(4)*t^3);
d_q2 = params(5) + (params(6)*t) + (params(7)*t^2) + (params(8)*t^3);
d_q1_dot = (params(2)) + (2*params(3)*t) + (3*params(4)*t^2); %first time derivative
d_q2_dot = (params(6)) + (2*params(7)*t) + (3*params(8)*t^2); %first time derivative
d_acc1 = (2*params(3)) + (6*params(4)*t); %second time derivative
d_acc2 = (2*params(7)) + (6*params(8)*t); %second time derivative
%control inputs for the joints
acc1 = (-K*[q1 - d_q1; q1_dot - d_q1_dot]) + d_acc1;
acc2 = (-K*[q2 - d_q2; q2_dot - d_q2_dot]) + d_acc2;
acc = [acc1;acc2];
%2-link Arm parameters, (given)
I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
g=9.8;
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;
%
M = [a+2*b*cos(z{2}), d+b*cos(z{2});
    d+b*cos(z{2}), d];
C = [-b*sin(z{2})*z{4}, -b*sin(z{2})*(z{3}+z{4}); b*sin(z{2})*z{3},0];
G = [m1*g*r1*cos(z{1})+m2*g*(l1*cos(z{1})+r2*cos(z{1}+z{2}));
    m2*g*r2*cos(z{1}+z{2})];
%
invM = inv(M);
q_dot_vec = [z{3};z{4}];
%Calculating torque
Tau = (M*acc) + (C*q_dot_vec) + G;
% Expression for qi_ddot 
qi_ddot = invM*(Tau - (C*q_dot_vec) - G);
%Defining the dynamics with first order ODEs
dzdt(1) = z{3};
dzdt(2) = z{4};
dzdt(3) = qi_ddot(1);
dzdt(4) = qi_ddot(2);
end