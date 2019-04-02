%% test_set_measurements_with_mst
clear; clc; close all;

%% Setup random data - no noise
% rand('seed', 24235235423); 
N = 4; 
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
E = [1 2; 2 3; 3 4; 4 1];
M = size(E,1);
w = ones(M, 1);
q_meas = zeros(4, M);
R_meas = zeros(3,3,M);

%% Add noise 
ang_noise_max = pi/4; %pi/4 %pi/192;
for idx=1:M
    R1 = Rs(:,:,E(idx,1));
    R2 = Rs(:,:,E(idx,2));
    dR = R1.'*R2*random_rot_matrix(3, ang_noise_max);
    qmi = bot_matrix_to_quat(dR);
    R_meas(:,:,idx) = dR;
    q_meas(:,idx) = qmi;
end

qm_out = set_measurements_with_mst(E, E(1:end-1,:), q_meas, [0; 1; 0; 0])

q_meas
