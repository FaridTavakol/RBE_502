%% RETURN JOINT SPACE ANGLES FOR A GIVEN END EFFECTOR POSITION

% Course: Robotic Manipulation and Mobility
% Advisor: Dr. V. Krovi
%
% Homework Number: 4
%
% Names: Sourish Chakravarty
% 	Hrishi Lalit Shah


% function to invert bot at time t and position x,y
% values always between -pi/3 and 2*pi/3
% risk of unfeasible space even with feasible solns when J is singular
% qdot = thdot and th1/th2 are the initial elbow up config. angles
function TH=invbot(X)
global l1 l2 %rx ry start_an ell_an w
l1=2; l2=1;
x=X(1);
y=X(2);
A=x^2+y^2+l1^2-l2^2+2*l1*x;
B=-4*l1*y;
C=x^2+y^2+l1^2-l2^2-2*l1*x;
th1=2*atan2(-B+sqrt(B^2-4*A*C),2*A); % using the eqn. (x-l1c1)^2+(y-l1s1)^2=l2^2
th2=atan2(y-l1*sin(th1),x-l1*cos(th1));
% xdot=-rx*sin(w*t+start_an)*cos(ell_an)*w-ry*cos(w*t+start_an)*sin(ell_an)*w;
% ydot=-rx*sin(w*t+start_an)*sin(ell_an)*w+ry*cos(w*t+start_an)*cos(ell_an)*w;
% J=[-l1*sin(th1) -l2*sin(th2);
%     l1*cos(th1)  l2*cos(th2)];
% qdot=inv(J)*[xdot;ydot]; % theta dot
TH=[th1 th2];% qdot'];