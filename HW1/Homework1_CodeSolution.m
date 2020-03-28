%% RBE 502 - Robot Control Homework 1 - by Alp Sahin
%% Clear & Close
clear all; close all; clc;

%% Problem 1
%
% Let's choose the generalized coordinates as theta and x, whereas x is the
% translation along the radial direction (along the spring). 

%%
%
% Defining the symbolic parameters:
syms x theta % generalized coordinates
syms x_dot theta_dot x_ddot theta_ddot % their derivatives
syms m g l_0 k % system parameters

%%
%
% Storing the system variables in corresponding vectors:
q = [x;theta];
q_dot = [x_dot;theta_dot];
q_ddot = [x_ddot;theta_ddot];


%% (a)
%
% Kinetic energy in the system is due:
% 
% # Rotation of the mass around the origin
% # Translation of the mass along the spring axis
% 

%%
%
% According to the definition of our generalized coordinates, total length
% of the pendulum: base length(l_0) + displacement amount(x).

l = l_0 + x; % total length of the pendulum
I = m*l^2; % Inertia of the mass around the origin
omega = theta_dot; % rotational velocity

% Kinetic Energy:
K = (1/2)*I*omega^2 + (1/2)*m*x_dot^2;

%%
%
% Potential energy in the system is due:
% 
% # Gravity
% # Spring

% Potential Energy:
P = (1/2)*k*x^2 - m*g*l*cos(theta);

%%
%
% Then the Lagrangian is:
L = K - P;

%%
%
% Derivation of the equations of motion in x and theta:
%
% Back and forth substitutions are necessary for differentiating the
% Lagrangian with respect to generalized coordinates and time. We need to
% define some redundant symbolic expressions to account for these substitutions.
syms x_dot_(t) theta_dot_(t) x_(t) theta_(t)
time_indep = [x;theta;x_dot;theta_dot];
time_dep = [x_(t);theta_(t);x_dot_(t);theta_dot_(t)];
diff_q = [diff(x_(t), t);diff(theta_(t), t)];
diff_q_dot = [diff(x_dot_(t), t);diff(theta_dot_(t), t)];

for i=1:length(q)
    % Differentiation of the Lagrangian wrt x_dot and theta_dot
    temp_L_a{i} = diff(L,q_dot(i));
    % Substitute the time independent expressions with time dependent
    % expressions before differentiation wrt time
    temp_L_b{i} = subs(temp_L_a{i},time_indep,time_dep);
    % Differentiation wrt time
    temp_L_c{i} = diff(temp_L_b{i});
    % Substitute time dependent terms and terms that include the expression
    % 'diff' with the regular terms defined in the beginning
    temp_L_d{i} = subs(temp_L_c{i},[diff_q;diff_q_dot;time_dep],[q_dot;q_ddot;q;q_dot]);
    % Differentiation of the Lagrangian wrt x and theta
    temp_L_e{i} = diff(L,q(i));
    % Obtain the equations of motion by subtraction:
    % (d/dt)(dL/dq_dot)-(dL/dq)
    eq{i} = temp_L_d{i} - temp_L_e{i};
    fprintf('equation %d=',i)
    disp(simplify(eq{i}))
end

% Solve the equations of motion for x_ddot and theta_ddot
sol = solve([eq{1}==0, eq{2}==0], q_ddot);

fprintf('x_ddot = ')
disp(simplify(sol.x_ddot))

fprintf('theta_ddot = ')
disp(simplify(sol.theta_ddot))

%% (b)
%
% Let's define the state variables:
X = [x;theta;x_dot;theta_dot];

%%
%
% Thus X_dot becomes [x_dot;theta_dot;x_ddot;theta_ddot]

%%
%
% Let's initialize the derivative of the state vector, X_dot:
X_dot = sym(zeros(4,1));

%%
%
% Expressing the state variables:
%
%   1. x_dot = X_dot(1)
X_dot(1) = X(3);

%%
%   2. theta_dot = X_dot(2)
X_dot(2) = X(4);

%%
%   3. x_ddot = X_dot(3)
X_dot(3) = sol.x_ddot;

%%
%   4. theta_ddot = X_dot(4)
X_dot(4) = sol.theta_ddot;

fprintf('X_dot =\n')
disp(X_dot)

%% (c)
%
% To find the equilibrium of the system let's substitute 0 for derivatives 
% of x and theta:
equilibrium = (subs(X_dot,q_dot,zeros(2,1)));

%%
%
% Then we solve the equations for x and theta to find the equilibrium
% state:
sol2 = solve([equilibrium(3)==0,equilibrium(4)==0],q);

fprintf('at equilibrium: \n')
fprintf('x = %s\n', sol2.x)
fprintf('theta = %d\n', sol2.theta)

%%
%
% As expected, theta = 0, which means that the pendulum is at the center
% and x = mg/k, which is the corresponding deflection of the spring under
% the weight of the mass.

%% (d)
%
% Since the mass will be restored to the equilibrium position, which is x =
% (m*g)/k and theta = 0, after any small stretching of the spring, swing of
% the pendulum or both, the equilibrium is stable.

%% (e)
%
% Let's simulate the system using the ode45 function with the initial 
% configurations of x = 4.5, x_dot = 0, theta = pi/18, theta_dot = 0. The
% ode function named ode_pendulum is attached at the end of the report.

% Defining a simulation time:
tspan = [0 20];

% Spring constant for this simulation:
k1 = 10;

% Simulation 1:
[t1,z1] = ode45(@(t,z) ode_pendulum(t,z,k1), tspan, [4.5,0,pi/18,0]);

% Equilibrium state for given parameters:
eq_x = subs(sol2.x,[m,g,l_0,k],[5,9.8,2,k1]);
eq_theta = subs(sol2.theta,[m,g,l_0,k],[5,9.8,2,k1]);

% Plots:
figure(1)
plot(t1,z1(:,1),'LineWidth',2)
hold on
grid on
plot(t1,ones(length(t1))*eq_x,'r--','LineWidth',2)
xlabel('Time(s)');
ylabel('x(m)');
ylim([3.5 6.3])
title(['k = ' num2str(k1)])
legend('system response','equilibrium')
hold off
snapnow

figure(2)
plot(t1,z1(:,3),'LineWidth',2)
hold on
grid on
plot(t1,ones(length(t1))*eq_theta,'r--','LineWidth',2)
xlabel('Time(s)');
ylabel('\theta(rad)');
ylim([-pi/2 pi/2])
title(['k = ' num2str(k1)])
legend('system response','equilibrium')
hold off
snapnow
%%
%
% Note: Plots are attached at the end of the document.
%
% From the figures we can see that both x and theta oscillate around the
% equilibrium point.

%
% Velocities in x and theta can also be plotted by uncommenting the
% following:
% 
% figure()
% plot(t1,z1(:,2),'LineWidth',2)
% plot(t1,z1(:,4),'LineWidth',2)
%

%% (f)
%
% By selecting a larger and a smaller spring constant we can observe the
% change in behavior depending on the spring constant.
%

% Spring constant for simulation 2:
k2 = 12;

% Simulation 2:
[t2,z2] = ode45(@(t,z) ode_pendulum(t,z,k2), tspan, [4.5,0,pi/18,0]);

% Spring constant for simulation 3:
k3 = 8;

% Simulation 3:
[t3,z3] = ode45(@(t,z) ode_pendulum(t,z,k3), tspan, [4.5,0,pi/18,0]);

% Plots:
figure(3)
plot(t1,z1(:,1),'LineWidth',2)
xlabel('Time(s)');
ylabel('x(m)');
ylim([3 8])
hold on
grid on
plot(t2,z2(:,1),'LineWidth',2)
plot(t3,z3(:,1),'LineWidth',2)
legend(['k = ' num2str(k1)],['k = ' num2str(k2)],['k = ' num2str(k3)])
title('Spring Constant Comparison')
hold off
snapnow

figure(4)
plot(t1,z1(:,3),'LineWidth',2)
xlabel('Time(s)');
ylabel('\theta(rad)');
ylim([-pi/2 pi/2])
hold on
grid on
plot(t2,z2(:,3),'LineWidth',2)
plot(t3,z3(:,3),'LineWidth',2)
legend(['k = ' num2str(k1)],['k = ' num2str(k2)],['k = ' num2str(k3)])
title('Spring Constant Comparison')
hold off
snapnow

%%
%
% By observing the comparison plots, we can see that increasing the spring
% constant results in a decrease in the oscillation amplitude while increasing
% the oscillation frequency, as expected. Another observation is that, the
% equilibrium point in x gets larger as the spring constant is decreased,
% which can also be confirmed by the equilibrium point solution (x=mg/k).


%% Problem 2

%% 1.
%
% Let's define the state vector X:
%
% X = [x;x_dot]
%
% Then the derivative is:
%
% X_dot = [x_dot;x_ddot]
%
% State Space Form: 
%
% X_dot = A*X;
%
% where,
%
% A = [0 1;-10 5]

%% 2.
%
% At equilibrium: x_dot = x_ddot = 0
%
% Therefore x = 0

%% 3.
%
% Equilibrium is unstable, because the roots of the equation lie on the
% right half plane.


%% ODE Function
%
% ode function used in Problem 1 is provided below:
%
function dz = ode_pendulum(t,z,k)
% System parameters
l_0 = 2; m = 5; g = 9.8;
dz = zeros(4,1);
z = num2cell(z);
[x, x_dot, theta, theta_dot] = deal(z{:});
if abs(theta) > 2*pi
    theta = mod(theta, 2*pi);
end
%
% Note that: z{1} = x, z{2} = x_dot, z{3} = theta, z{4} = theta_dot
%
dz(1) = z{2};
dz(2) = (g*m*cos(z{3}) - k*z{1} + l_0*m*z{4}^2 + m*z{4}^2*z{1})/m;
dz(3) = z{4};
dz(4) = -(2*z{4}*z{2} + g*sin(z{3}))/(l_0 + z{1});
end

