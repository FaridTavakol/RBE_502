function dz = ode_dubins(t,z)
% use z for [x,y,theta]
dz =zeros(3,1);

%% TODO: Here is the code for control input.
%%% the controller needs to provide control input: linear velocity and
%%% steering angle: v, delta

% step 1: calculate the value of deviation variables given the
% desired state and input and the actual state, exacted from z.
v_d = 10;
w_d = 0;
u_d = [v_d ; w_d];

y_d = 2;
theta_d = 0;
v_r = 10;
x_d = [v_r*t; y_d ;theta_d];
e_x = z - x_d;

% de_x = dx - dx_d;

% step 2: based on the feedback controller, calculate the e_u=
% [e_v, e_w].
k=[5000 200 300;
    1000 200 300];
e_u = -k*e_x; 

% step 3: based on the relation between u and e_u, derive the
% desired input v, and w.
% e_u= u - u_d; --> u = e_u + u_d
u= -k*e_x + u_d;
v= u(1);
w= u(2);
theta= z(3);

dz(1) = v*cos(theta);
dz(2) = v*sin(theta);
dz(3) = w;
end