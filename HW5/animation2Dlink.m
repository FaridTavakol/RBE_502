
%% Animation of the TWO Link Manipulator under PD Controller %%
%PD Animation%
%Initialization of Tip point of every Link%
function [] = animation2Dlink(X, l1, l2)
% input: X is a list of joint states, 
% X(i, 1): The joint variable q_1
% X(i, 2): The joint variable q_2
% l1: the length of the first link.
% l2: the length of the second link.

k=size(X,1);
pointl1=zeros(k,2); %End Point of Link1
pointl2=zeros(k,2); %End Point of Link2

%Create Figure for the Animation%
figure('Name', 'Animation of the 2 Link Manpiulator under PD Control')
movegui(2,'northwest')

%Computing the Tip Point positions corresponding to theta1, theta2%
for ii=1:k

%Adjusting Figure for every iteration%
clf
xlabel('X - POSITION') 
ylabel('Y - POSITION') 
grid on
title('TWO LINK MANIPULATOR ANIMATION UNDER Task Space PD CONTROL');
axis([floor(min(pointl2(:,1))) 2.2 -2.2 2.2]);
axis square

%Computing the points%
pointl1(ii,1) =  l1*cos(X(ii,1)) ;     %Computing x positions of end point of link 1
pointl1(ii,2) = l1*sin(X(ii,1));       %Computing y positions of end point of link 1
pointl2(ii,1) = pointl1(ii,1) + (l2*cos(X(ii,1)+X(ii,2))); %Computing x positions of end point of link 2
pointl2(ii,2) = pointl1(ii,2)+(l2*sin(X(ii,1)+X(ii,2)));   %Computing y positions of end point of link 2

%Plotting the links%
line([0,pointl1(ii,1)],[0,pointl1(ii,2)],'linewidth',2,'color','black');
line([pointl1(ii,1),pointl2(ii,1)],[pointl1(ii,2),pointl2(ii,2)],'linewidth',2,'color','blue');

hold on

plot(0,0,'o','markersize',7)
plot(pointl1(ii,1),pointl1(ii,2),'o','markersize',7)
plot(pointl2(ii,1),pointl2(ii,2),'o','markersize',7)
plot(pointl2(:,1),pointl2(:,2),'-')
pause(.01) 

end

