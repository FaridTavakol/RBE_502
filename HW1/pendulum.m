% inverted pendulum control: Example

syms x theta F x_dot theta_dot x_ddot theta_ddot g
syms M m l 
X  = sym ('X', [4,1]); % create a 4 by 1 vector for state
X(1)=x;
X(2)= x_dot;
X(3) = theta;
X(4) = theta_dot;

eq1 = (M+m)*x_ddot - m*l*theta_ddot*cos(theta) + m*l*theta_dot^2 *sin(theta)-F;
eq2 = l*theta_ddot - g*sin(theta)  - x_ddot *cos(theta);

sol = solve([eq1==0, eq2==0], [x_ddot, theta_ddot]);

% display the solution using the following commands:
% sol.x_ddot
%  
% ans =
%  
% (- l*m*sin(theta)*theta_dot^2 + F + g*m*cos(theta)*sin(theta))/(M + m - m*cos(theta)^2)
%  
% sol.theta_ddot
%  
% ans =
%  
% (- l*m*cos(theta)*sin(theta)*theta_dot^2 + F*cos(theta) + g*m*sin(theta) + M*g*sin(theta))/(l*(M + m - m*cos(theta)^2))


