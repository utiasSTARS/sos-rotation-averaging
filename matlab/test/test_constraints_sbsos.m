%% test_constraints_sbsos.m
clear; close all; %clc;
eq_tol = 1e-8;

%% Setup random data - no noise
% rand('seed', 1);
N = 1000; %3;
Rs = zeros(3,3,N);
qs = zeros(4*N, 1);
% Use this matrix to avoid identity which is on the threshold of the double
% cover for quaternions in terms of q_x. Also avoid 180 rotations!
Rs(:,:,1) = [1 0 0; 0 0 -1; 0 1 0]; %eye(3);

%% Warning: error in bot_matrix_to_quat for 180 rotation!
qs(1:4) = bot_matrix_to_quat(Rs(:,:,1)); %[0; 1; 0; 0];
for idx=2:N %2:N
    Rs(:,:,idx) = random_rot_matrix(3);
    qsi = bot_matrix_to_quat(Rs(:,:,idx));
    % Only use positive x half of the double cover
    if qsi(2) < 0
        qsi = -qsi;
    end
    qs(4*(idx-1)+1:4*(idx-1)+4) = qsi;
end
qs_mat = reshape(qs, 4, []);
E = [1 2; 2 3]; %3 1]; % 3 1]; % 3 1];
M = size(E,1);
q_meas = zeros(4, M);

for idx=1:M
    R1 = Rs(:,:,E(idx,1));
    R2 = Rs(:,:,E(idx,2));
    dR = R1.'*R2;
    qmi = bot_matrix_to_quat(dR);
    % Should this affect cost in F? It seems to because for identity
    % quaternion q_x = 0, huge ambiguity 
    if qmi(2) < 0
        qmi = -qmi;
    end
    q_meas(:,idx) = qmi;
end

%% Test G (constraint functions)
I = get_I_quaternion(E, true);
G = get_G_quaternion(N, qs(1:4), I);
G2 = get_G_quaternion_double_cover(N, qs(1:4), I);
for idx=1:length(G)
%     idx
    val_idx = evalpoly(G{idx}, qs);
    assert(val_idx >= -10*eps);
    assert(val_idx <= 1+10*eps);
end

for idx=1:length(G2)
%     idx
    val_idx = evalpoly(G2{idx}, qs);
    assert(val_idx >= -10*eps);
    assert(val_idx <= 1+10*eps);
end