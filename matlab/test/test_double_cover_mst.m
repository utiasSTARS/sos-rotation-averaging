%% test_double_cover_mst.m
clear; close all; clc;
eq_tol = 1e-8;

%% Solver params
mle = false;
clean = true;
double_cover = true;

%% Setup random data - no noise
rand('seed', 242571); 
N = 7;
Rs = zeros(3,3,N);
qs = zeros(4*N, 1);
Rs(:,:,1) = [1 0 0; 0 0 -1; 0 1 0];  %eye(3);
qs(1:4) = bot_matrix_to_quat(Rs(:,:,1)); %[0; 1; 0; 0];
ang_max = pi;
for idx=2:N
    Rs(:,:,idx) = Rs(:,:,idx-1)*random_rot_matrix(3, ang_max);
    qsi = bot_matrix_to_quat(Rs(:,:,idx));
    qs(4*(idx-1)+1:4*(idx-1)+4) = qsi;
end
qs_mat = reshape(qs, 4, []);

%% Add edges
n_loop_closures = 1;
% E = [1 2; 2 3; 3 1];
E = [1 2; 2 3; 3 4; 4 5; 5 6; 6 7; 7 1; 6 2];

M = size(E,1);
w = ones(M, 1);
q_meas = zeros(4, M);
R_meas = zeros(3,3,M);

%% Create and add noise to all edges
ang_noise_max = 0; %pi/4; %pi/2 %pi/192;
for idx=1:M
    R1 = Rs(:,:,E(idx,1));
    R2 = Rs(:,:,E(idx,2));
    dR = R1.'*R2*random_rot_matrix(3, ang_noise_max);
    qmi = bot_matrix_to_quat(dR);
    R_meas(:,:,idx) = dR;
    q_meas(:,idx) = qmi;
end


%% Add noise to one edge
ang_noise_val = pi; %pi/192;
R1 = Rs(:,:,E(1,1));
R2 = Rs(:,:,E(1,2));
% dR = R1.'*R2*random_rot_matrix(3, ang_noise_max);
a = rand(3,1);
a = a/norm(a);
dR = R1.'*R2*axis_angle(a, ang_noise_val);
qmi = bot_matrix_to_quat(dR);
R_meas(:,:,1) = dR;
q_meas(:,1) = qmi;


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
        % TODO: fix it up
%         assert(verify_rip(pop.I));
    end
    
    obj_val(idx) = psol.obj;
end

figure;
histogram(obj_val, 'BinWidth', 0.0001);
grid on;
title('Obj. Val over runs');


%% Try MSTs as seeds
% mst1 = E(1:6, :);
% mst1 = [1 2; 4 1; 3 1];
mst1 = E(1:6,:);
qm_mst1 = set_measurements_with_mst(E, mst1, q_meas, [0; 1; 0; 0]);

[~, psol1, ~] = rotation_averaging_sbsos(qm_mst1, R_meas, E, qs(1:4), ...
                                       clean, w, double_cover, mle);
mst2 = E(2:7,:);
qm_mst2 = set_measurements_with_mst(E, mst2, q_meas, [0; 1; 0; 0]);
                
[~, psol2, ~] = rotation_averaging_sbsos(qm_mst2, R_meas, E, qs(1:4), ...
                                       clean, w, double_cover, mle);
                                   
                                   

%% Save
% save_string = ['../results/mst_double_cover_experiment_N_' num2str(N)];
% save_string = [save_string '_n_loop_' num2str(n_loop_closures)];
% save(save_string);