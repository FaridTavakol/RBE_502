%%% FUNCTION FOR PLOTTING THE SIMULATED OUTPUT FOR A GIVEN TIME VECTOR AND
%%% JOINT SPACE ANGLE VECTORS

% Course: Robotic Manipulation and Mobility
% Advisor: Dr. V. Krovi
%
% Homework Number: 4
%
% Names: Sourish Chakravarty
% 	Hrishi Lalit Shah

function plotbot_js(t,X,index,txt1)
global l1 l2 rx ry ell_an start_an w A B
aviobj = avifile([txt1,'.avi'],'compression','Cinepak'); % Declare an avi object

h=figure(index*3-2);
cla('reset');
axis manual;
axis([-3 3 -3 3]);
hold on;
grid on;
c_ell_an=cos(ell_an);
s_ell_an=sin(ell_an);
plot(A+rx*cos(w*t)*c_ell_an-ry*sin(w*t)*s_ell_an,...
     B+rx*cos(w*t)*s_ell_an+ry*sin(w*t)*c_ell_an,'-k');
title(txt1);
for i=1:length(t)
    th1=X(i,1);
    th2=X(i,2);
    x1=l1*cos(th1);
    y1=l1*sin(th1);
    x2=x1+l2*cos(th2);
    y2=y1+l2*sin(th2);
%     plot([0 x1 x2],[0 y1 y2]);
    plot(x2,y2,'ro');
%     pause(0.03);
    pause(0.1);     %Stop execution for 0.01 to make animation visible
    frame= getframe(gcf);   %Step 2: Grab the frame
    aviobj = addframe(aviobj,frame); % Step 3: Add frame to avi object
end
aviobj = close(aviobj)  % Close the avi object
hold off
% axis equal;