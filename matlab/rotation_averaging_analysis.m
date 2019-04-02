%% Analysis of rotation averaging (no translation)
clear; close all; clc;

%% Show SOS-Convexity? 

% Quaternions have q.'*q = 1, which is known to be SOS-convex
% The cost function is quadratic, just need to check the Hessian!
qij = sym('qij', [4 1]);

% But we DON'T need to! The hessian will just be the sum of the terms'
% Hessians which are of the form A.'*A, so we know it's SOS-convex! 

