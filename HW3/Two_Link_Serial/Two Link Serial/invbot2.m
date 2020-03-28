%% RETURN JOINT SPACE ANGLES FOR A GIVEN END EFFECTOR POSITION AND AN
%% INITIAL START VALUE FOR JOINT SPACE ANGLES

% Course: Robotic Manipulation and Mobility
% Advisor: Dr. V. Krovi
%
% Homework Number: 4
%
% Names: Sourish Chakravarty
% 	Hrishi Lalit Shah


%% function invbot with 2pi additions till value is nearest to initial
%% approx. works best with all conditions. plan to use this.
% can give both q as th1/th2 and qdot as thetadot. for thd, include t as
% parameter.
function TH=invbot2(X,th)
global l1 l2%rx ry start_an ell_an w
l1=2; l2=1;
x=X(1);
y=X(2);
th1a=th(1);
th2a=th(2);
A=x^2+y^2+l1^2-l2^2+2*l1*x;
B=-4*l1*y;
C=x^2+y^2+l1^2-l2^2-2*l1*x;
th1=2*atan2(-B+sqrt(B^2-4*A*C),2*A)-2*pi;
th2=atan2(y-l1*sin(th1),x-l1*cos(th1))-2*pi;
prev=100;
while(abs(th1-th1a)<prev)
    prev=abs(th1-th1a);
    th1=th1+2*pi;
end
prev=100;
while(abs(th2-th2a)<prev)
    prev=abs(th2-th2a);
    th2=th2+2*pi;
end
% xdot=-rx*sin(w*t+start_an)*cos(ell_an)*w-ry*cos(w*t+start_an)*sin(ell_an)*w;
% ydot=-rx*sin(w*t+start_an)*sin(ell_an)*w+ry*cos(w*t+start_an)*cos(ell_an)*w;
% J=[-l1*sin(th1) -l2*sin(th2);
%     l1*cos(th1)  l2*cos(th2)];
% qdot=inv(J)*[xdot;ydot];
TH=[th1-2*pi th2-2*pi]';% qdot'];