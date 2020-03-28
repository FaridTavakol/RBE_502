clc; clear;
A=zeros(3);
A(2,3)=10;
B=zeros(3,2);
B(1,1)=1;
B(3,2)=1;
k=[5000 200 300;
    1000 200 300];
p=A-(B*k)
desired_poles=eig(p)

place(A,B,desired_poles)