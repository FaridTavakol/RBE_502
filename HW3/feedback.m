%%This function has the state space representation of the dynamics model of the two link arm
function [ dx ] = feedback( t,x)

        q_d =[0;0];
        
        dq_d=[0;0];
        
        ddq_d=[0;0]; 
       

        q= x(1:2,1);
        dq= x(3:4,1);
        
        % the following parameters for the arm
        I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
        g=9.8;   
        a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
        b = m2*l1*r2;
        d = I2+ m2*r2^2;
        % the actual dynamic model of the system:
        Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
        Gmat =  [m1*g*r1*cos(x(1))+m2*g*(l1*cos(x(1))+r2*cos(x(1)+x(2)));
        m2*g*r2*cos(x(1)+x(2))];
        invM = inv(Mmat);
        invMC = invM*Cmat;

        
        %Initialising the K values
       
        Kp1 = 100;
        Kp2 = 250;
        Kd1 = 50;
        Kd2 = 200;
        
        Kp = [Kp1, 0;
              0, Kp2];
        Kd = [Kd1, 0;
              0, Kd2];

        %Calculating the errors
        e = [q(1) - q_d(1); q(2)- q_d(2)];
        de = [dq(1) - dq_d(1); dq(2) - dq_d(2)];
        
        Acc_d = ddq_d; 
        

        % Inverse dynamics (Computed Torque Control(CTC))controller
        Acc = Acc_d -Kp*e -Kd*de; %Calculating the actual acceleration
        %Substitute Acc to get the torque input Tau(u) to the system
        Tau = Mmat*Acc + Cmat*dq_d + Gmat;

        
        % use the computed torque and state space model to compute
        % the increment in state vector.

        %% State Space Form
        dx = zeros(4,1);
        dx(1) = x(3); % state matrix is [q1 q2 dq1 dq2]
        dx(2) = x(4);        
        dx(3:4,:) = invM*(Tau - Cmat*dq - Gmat);
         
end



