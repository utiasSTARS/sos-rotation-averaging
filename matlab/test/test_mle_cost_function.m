%% test_mle_cost_function
clear; close all; clc;

%% Params
eq_tol = 1e-8;
%% Setup random data - no noise
% rand('seed', 1);
% rand('seed', 6); % Works when ang_max = pi!
N = 6; %3;
Rs = zeros(3,3,N);
qs = zeros(4*N, 1);
Rs(:,:,1) = [1 0 0; 0 0 -1; 0 1 0];  %eye(3);
qs(1:4) = bot_matrix_to_quat(Rs(:,:,1)); %[0; 1; 0; 0];
ang_max = pi/12;
for idx=2:N %2:N
    Rs(:,:,idx) = Rs(:,:,idx-1)*random_rot_matrix(3, ang_max);
    qsi = bot_matrix_to_quat(Rs(:,:,idx));
    % Only use positive x half of the double cover
    if qsi(2) < 0
        qsi = -qsi;
    end
    qs(4*(idx-1)+1:4*(idx-1)+4) = qsi;
end
qs_mat = reshape(qs, 4, []);

%% Add edges
n_loop_closures = 2;
E = make_random_pose_graph(N, n_loop_closures);

M = size(E,1);
q_meas = zeros(4, M);
R_meas = zeros(3,3,M);
%% Add noise
ang_noise_max = pi/24; %pi/192;
for idx=1:M
    R1 = Rs(:,:,E(idx,1));
    R2 = Rs(:,:,E(idx,2));
    dR = R1.'*R2*random_rot_matrix(3, ang_noise_max);
    R_meas(:,:,idx) = dR;
    qmi = bot_matrix_to_quat(dR);
    % Should this affect cost in F? It seems to because for identity
    % quaternion q_x = 0, huge ambiguity 
    if qmi(2) < 0
        qmi = -qmi;
    end
    q_meas(:,idx) = qmi;
end

%% Test cost functions
f_gt = mle_loss(R_meas, E, Rs)
f_gt_quat = mle_loss_quat(R_meas, E, qs_mat)
F = get_F_mle(R_meas,E);
f_matrix = evalpoly(F,qs)