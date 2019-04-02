clear; close all; clc;
clear all; clear all;
eq_tol = 1e-8;
%% Setup random data - no noise
rand('seed', 11);
Ns = randi(20,100,1) + 5;

%% Setup pose graph

for k = 1 : size(Ns,1)
    N = Ns(k); % number of poses
    n_loop_closures = 3; % loop closures
    E = make_random_pose_graph(N, n_loop_closures); % makes a random graph
    
    I = get_I(E,true);
    
    if ~verify_rip(I)
        disp("RIP doesn't hold")
        break
    end
end

if k == size(Ns,1)
    disp("RIP held!")
end