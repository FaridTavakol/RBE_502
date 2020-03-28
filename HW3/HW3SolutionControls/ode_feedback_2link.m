function dzdt = ode_feedback_2link(t,z)
%Defining the A,B and K matrices in the equation: x_dot = Ax + Bacc
%And: acc = -Kx
A = [0 1; 0 0];
B = [0; 1];
K = place(A,B,[-3 -5]); %eigenvalues -3, -5
%Defining the output time derivative of state: z_dot
dzdt = zeros(4,1);
z = num2cell(z);
[q1, q2, q1_dot, q2_dot] = deal(z{:});
%Checking bounds of joint angles
if abs(q1) > 2*pi
q1 = mod(q1, 2*pi);
end
if abs(q2) > 2*pi
q2 = mod(q2, 2*pi);
end
%ai = -Kxi
acc1 = -K*[q1; q1_dot];
acc2 = -K*[q2; q2_dot];
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
%Calculating torque from dynamic equation: M*acc + C*q_dot + G = Tau
Tau = (M*acc) + (C*q_dot_vec) + G;
% Expression for qi_ddot/acc 
qi_ddot = invM*(Tau - (C*q_dot_vec) - G);
%Defining the dynamics with first order ODEs
dzdt(1) = z{3};
dzdt(2) = z{4};
dzdt(3) = qi_ddot(1);
dzdt(4) = qi_ddot(2);
end