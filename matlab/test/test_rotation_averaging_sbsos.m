%% test_rotation_averaging_sbsos.m
clear; close all; %clc;
eq_tol = 1e-8;
%% Setup random data - no noise
% rand('seed', 1);
%rand('seed', 6); % Works when ang_max = pi!
N = 5; %3;
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
% Problem appears to manifest when there's a small loop or a repetition of
% a constraint. 
% Also appears probabilistic! Depends on the data! Or perhaps some random
% sampling of the data?
%E = [1 2; 2 3; 3 4; 4 5; 4 1]; %4 5]; %2 3]%; 3 1]; % 3 1]; % 3 1];

%% Add edges
n_loop_closures = 1;
E = make_random_pose_graph(N, n_loop_closures);
%E = [1 2; 2 3; 3 4; 4 1];

M = size(E,1);
q_meas = zeros(4, M);
R_meas = zeros(3,3,M);
%% Add noise
ang_noise_max = 0;% pi/192;
for idx=1:M
    R1 = Rs(:,:,E(idx,1));
    R2 = Rs(:,:,E(idx,2));
    dR = R1.'*R2*random_rot_matrix(3, ang_noise_max);
    qmi = bot_matrix_to_quat(dR);
    R_meas(:,:,idx) = dR;
    % Should this affect cost in F? It seems to because for identity
    % quaternion q_x = 0, huge ambiguity 
    if qmi(2) < 0
        qmi = -qmi;
    end
    q_meas(:,idx) = qmi;
end

% Not sure this has any way of selecting the correct q_i from the inputted
% q_ij variables, the q_ijs may just screw us off the bat (50/50 chance of
% being the correct orientation for arbitrary relative poses!)
% But doesn't multiplication work no matter which (+ve or -ve) quaternion 
% is used? Yes, but may map to either the positive or negative version.

mle = true;
clean = true;
% If double_cover == true, DON'T use q_xi > 0
double_cover = false;
w = ones(M, 1);
[sol, psol, pop] = rotation_averaging_sbsos(q_meas, R_meas, E, qs(1:4), ...
                                       clean, w, double_cover, mle);
psol
