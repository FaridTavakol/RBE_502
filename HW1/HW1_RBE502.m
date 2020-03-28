%% Farid Tavakkolmoghaddam
% HW1 Problem 1
clear all;
k=10; %stifness coefficient
l=2; % initial length
m=5; % mass 
% m2=5;
x=3.5; % initial deflection of the spring  
p=10; % initial deflection angle of the spring
g=9.81; % gravitational acceleration
y10=[p/360*2*pi 0 x 0]; % Initial Conditions X=[ x1 x1' theta theta']

tspann=[linspace(0,40,201)];% Duration
 f=@(t,y)[y(2) ; ...
         (-2*m*(l+y(3))*y(4)*y(2)-m*g*(l+y(3))*sin(y(1)))/(m*(l+y(3))^2); ...
        y(4) ; ...
        (l+y(3)*m*y(2)^2+m*g*cos(y(1))-k*y(3))/m]; 

[t,y]=ode45(f,tspann,y10);

x2=(l+y(:,3)).*sin(y(:,1));
y2=(l+y(:,3)).*cos(y(:,1));


for k=1:1:1
    figure(1)
for i=1:1:length(t)
    hold on;
    axis([-12 12 -12 12]);
   
    title('Spring Pendulum');
    plot( [0 x2(i) ],-[0 y2(i)] ,'k -');
    
    plot(x2(1:i),-y2(1:i),'b :');
    
    plot(x2(i),-y2(i),'r o');% massenpunkt m2
    plot([-5 5],[0 0],'k -');
    plot(x2(i),-y2(i),'r o');
    if i==length(t)
        break
    end
    drawnow;
    clf;
end
hold off
end
