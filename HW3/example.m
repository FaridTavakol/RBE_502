clc ;clear all

A=[0 1;0 0];
B=[0;1];
syms q1 dq1 k1 k2 q1_d dq1_d
Z=A*[q1;dq1]+B*[-k1 -k2]*[q1-q1_d;dq1-dq1_d]
