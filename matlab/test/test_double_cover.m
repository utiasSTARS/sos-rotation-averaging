%% test_double_cover.m
clear; close all; %clc;
eq_tol = 1e-8;

%% Thoughts/results
% The assignment of edge orientation really matters for ang_noise_max=pi/4.
% For noise free case, it's almost worse!! Not solved in many cases, need
% to understand why this is so. Need to check these cases, are they
% corresponding to MSTs? 
%     6
%     11
%     19
%     30
%     36
%     45
%     53
%     60
%     66
%     79
%     87
%     90
%    104
%    105
%    113
%    128

% Also, there are a discrete number of objective values (including ones
% where the solution could not be extracted). In cases where the solution
% could not be extracted, does this go against the theory of Sparse-BSOS? I
% don't think so because it only guarantees that a solution will be optimal
% if the conditions are met, not that there is a unique optimal condition. 
% 
% Looking for some graphical notion of sharing the residuals that applies
% to the double cover case.

% For N vertices, size of obj. value groupings are always multiples of 
%2^(N-1), i.e. the orientations chosen for some MST). 
%
% The number of obj. value groupings seems to always be 2^(M-(N-1)), where 
% M is the number of measurements (edges). This is equivalent to the number
% of ways of assigninng +/- to the number of non-MST edges.
%

%% Solver params
mle = false;
clean = true;
double_cover = true;

%% Setup random data - no noise
% rand('seed', 12345678);
rand('seed', 24235235423); 
N = 5; 
Rs = zeros(3,3,N);
qs = zeros(4*N, 1);
Rs(:,:,1) = [1 0 0; 0 0 -1; 0 1 0];  %eye(3);
qs(1:4) = bot_matrix_to_quat(Rs(:,:,1)); %[0; 1; 0; 0];
ang_max = pi/6;
for idx=2:N
    Rs(:,:,idx) = Rs(:,:,idx-1)*random_rot_matrix(3, ang_max);
    qsi = bot_matrix_to_quat(Rs(:,:,idx));
    qs(4*(idx-1)+1:4*(idx-1)+4) = qsi;
end
qs_mat = reshape(qs, 4, []);

%% Add edges
n_loop_closures = 1;
% E = make_random_pose_graph(N, n_loop_closures);
% E = [1 2; 2 3; 3 4; 4 5; 1 5; 2 5; 3 5];
% E = [1 2; 2 3; 3 4; 4 5; 1 5];
E = [1 2; 2 3; 3 4; 4 1; 3 1];
% E = [1 2; 2 3; 3 4; 2 4];

M = size(E,1);
w = ones(M, 1);
q_meas = zeros(4, M);
R_meas = zeros(3,3,M);
%% Add noise
ang_noise_max = pi/2; %pi/4 %pi/192;
for idx=1:M
    R1 = Rs(:,:,E(idx,1));
    R2 = Rs(:,:,E(idx,2));
    dR = R1.'*R2*random_rot_matrix(3, ang_noise_max);
    qmi = bot_matrix_to_quat(dR);
    R_meas(:,:,idx) = dR;
    q_meas(:,idx) = qmi;
end

%% Iterate over all measurement ambiguities 
f_val = zeros(1, 2^M);
obj_val = zeros(size(f_val));
is_solved = zeros(size(f_val));
for idx=1:2^M
    bit_string = pad(dec2bin(idx-1), M, 'left', '0');
    q_meas_idx = zeros(size(q_meas));
    for jdx=1:length(bit_string)
        if bit_string(jdx) == '1'
            q_meas_idx(:,jdx) = q_meas(:,jdx);
        else
            q_meas_idx(:,jdx) = -q_meas(:,jdx);
        end
    end
    [sol, psol, pop] = rotation_averaging_sbsos(q_meas_idx, R_meas, E, qs(1:4), ...
                                       clean, w, double_cover, mle);
    if psol.rnk == 1
        is_solved(idx) = 1;
        f_val(idx) = psol.fx;
    else
        f_val(idx) = -10;
    end
    if idx == 1
        assert(verify_rip(pop.I));
    end
    
    obj_val(idx) = psol.obj;
end

figure;
histogram(obj_val, 'BinWidth', 0.0001);
grid on;
title('Obj. Val over runs');

%% Save
% save_string = ['../results/simple_double_cover_experiment_N_' num2str(N)];
% save_string = [save_string '_n_loop_' num2str(n_loop_closures)];
% save(save_string);