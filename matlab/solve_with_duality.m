clear; close all; clc;
clear all; clear all;
eq_tol = 1e-8;
%% Setup random data - no noise
% rand('seed', 1);
rand('seed', 66348723052462346);

%% Setup pose graph
N = 20; % number of poses
n_loop_closures = 1; % loop closures
E = make_random_pose_graph(N, n_loop_closures); % makes a random graph
M = size(E,1); % edges

Rs = zeros(3,3,N); % rot matrices for every pose
qs = zeros(4*N, 1); % quaternions for every pose
Rs(:,:,1) = [1 0 0; 0 0 -1; 0 1 0];  %eye(3);
qs(1:4) = bot_matrix_to_quat(Rs(:,:,1)); %[0; 1; 0; 0]; Why bot?
ang_max = pi/12; % maximum angular displacement between poses I think

for idx=2:N %2:N
    Rs(:,:,idx) = Rs(:,:,idx-1)*random_rot_matrix(3, ang_max); % random new pose
    qsi = bot_matrix_to_quat(Rs(:,:,idx)); % quat from that pose
    % Only use positive x half of the double cover
    if qsi(2) < 0
        qsi = -qsi; % make it cool
    end
    qs(4*(idx-1)+1:4*(idx-1)+4) = qsi; % turn into vector
end

qs_mat = reshape(qs, 4, []); % readable matrix form


%% Generate measurements
q_meas = zeros(4, M); % empty measurements  quat
R_meas = zeros(3,3,M); % empty measurements rot
ang_noise_max = 0; % the noise is probably uniform on +- ang_max

for idx=1:M
    R1 = Rs(:,:,E(idx,1));
    R2 = Rs(:,:,E(idx,2));
    dR = R1.'*R2*random_rot_matrix(3, ang_noise_max);
    qmi = bot_matrix_to_quat(dR);
    R_meas(:,:,idx) = dR;
    %             if qmi(2) < 0
    %                 qmi = -qmi;
    %             end
    q_meas(:,idx) = qmi;
end

%% Generate matrices for the problem
A = zeros(4*M,4*(N-1));
b = zeros(4*M,1);

for m = 1:M % cycle through edges
    Qij = quatmultMatrixRight(q_meas(:,m));
    ind_q_i = E(m,1);
    ind_q_j = E(m,2);
    ind_A_m = 4*m - 3 : 4*m; % row in A matrix
    ind_A_i = 4*ind_q_i - 7 : 4*ind_q_i - 4; % columns corresponding to qi
    ind_A_j = 4*ind_q_j - 7 : 4*ind_q_j - 4; % columns corrsponding to qj
    
    if ind_q_i ~= 1 && ind_q_j ~= 1
        A(ind_A_m, ind_A_i) = Qij;
        A(ind_A_m, ind_A_j) = -eye(4);
    else
        ind_b_m = ind_A_m;
        if ind_q_i == 1 % if we have measurements starting from the initial state we dump that in the b matrix
            b(ind_b_m) = -Qij*qs_mat(:,ind_q_i);
            A(ind_A_m, ind_A_j) = -eye(4);
        else  % if measurement closing on anchor
            b(ind_b_m) = qs_mat(:,ind_q_j);
            A(ind_A_m, ind_A_i) = Qij;
        end
    end
end

%% Solve primal using plain cvx as test
disp(norm(A*qs(5:end)-b)^2)
A_big = [A'*A, - A'*b; -b'*A, b'*b];
%disp(trace(A_big*(q_hh*q_hh')))
%disp(f_quaternion_loss(q_meas, E, qs_mat))

cvx_begin
    variable q(4*(N-1)) 
    minimize norm(A*q-b,2)   
cvx_end

q_prim = q;
q_prim_mat = reshape(q, 4, []);

%disp(vecnorm(q_prim_mat,2))
%disp(get_abs_error(q_prim, qs(5:end)))
disp(f_quaternion_loss(q_meas, E, [qs_mat(:,1) q_prim_mat]))
disp(norm(A*q_prim-b)^2)
q_hh = [q_prim; 1];
disp(trace(A_big*(q_hh*q_hh')))

% normalize the solutions
for i = 1:N-1
    q_prim_norm((i-1)*4+1: (i-1)*4+4) = q_prim_mat(:,i)/ norm(q_prim_mat(:,i)) ;
end
q_prim_norm_mat = reshape(q_prim_norm, 4, []);
%disp(vecnorm(q_prim_norm_mat,2))
%disp(get_abs_error(q_prim_norm_mat, qs(5:end)))
disp(f_quaternion_loss(q_meas, E, [qs_mat(:,1) q_prim_norm_mat]))
disp(norm(A*q_prim_norm'-b)^2)
q_hh = [q_prim_norm'; 1];
disp(trace(A_big*(q_hh*q_hh')))

%% Solve using "dual of dual" - QCQP transformed to SDP
q_sol = duality_solver(q_meas, E, qs_mat(:,1));
q_sol_mat = reshape(q_sol, 4, []);

%disp(vecnorm(q_sol_mat,2))
%disp(get_abs_error(q_sol, qs(5:end)))
disp(f_quaternion_loss(q_meas, E, [qs_mat(:,1) q_sol_mat]))
disp(norm(A*q_sol-b)^2)
q_hh = [q_sol; 1];
disp(trace(A_big*(q_hh*q_hh')))