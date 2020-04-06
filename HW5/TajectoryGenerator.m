function  b = TajectoryGenerator( q1_0, dq1_0, q1_f, dq1_f,t0,tf)
a=[1 t0 t0^2 t0^3;
    0 1 2*t0 3*t0^2;
    1 tf tf^2 tf^3;
    0 1 2*tf 3*tf^2];
c=[q1_0 dq1_0 q1_f dq1_f].';
% For finding the unknowns for the polynomial coefficients
%if we have an equation like a*x=c > x=inv(a)*c 
b= a\c;
t = t0:0.01:tf;
posTraj = b(1)+b(2)*t+ b(3)*t.^2+b(4)*t.^3;
velTraj = b(2)*t+ 2*b(3)*t +3*b(4)*t.^2;
accTraj = 2*b(3) +6*b(4)*t;

% figure('Name','Trajectories for two link arm ');
% plot(t, posTraj,'b--');
% title('Desired trajectories')
% xlabel('Time (s)')
% ylabel('Position (degree)')
% grid on
% 
% figure('Name','Desired velocity for two link arm ');
% plot(t, velTraj,'b--');
% title('Desired velocity')
% xlabel('Time (s)')
% ylabel('Velocity (degree/s)')
% grid on
% figure('Name','Trajectories for two link arm ');
% plot(t, accTraj,'b--');
% title('Desired acceleration')
% xlabel('Time (s)')
% ylabel('Acceleration (degree/s^2)')
% grid on

end