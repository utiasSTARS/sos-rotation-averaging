%% Quaternion sign ambiguity experiments
clear; close all; clc;

%% First 3-pose case

q1 = rand(4,1)*2 - 1;
q1 = q1/norm(q1);
q2 = rand(4,1)*2 - 1;
q2 = q2/norm(q2);
q3 = rand(4,1)*2 - 1;
q3 = q3/norm(q3);


q12 = inv(quatmultMatrixLeft(q1))*q2;