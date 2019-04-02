%% test_rip_algorithms.m
clear; close all; clc;
rand('seed', 248163264);
%% Params
n_runs = 1000;
rip_clique = zeros(1, n_runs);
rip_fast = zeros(size(rip_clique));
is_solved = zeros(size(rip_clique));
t_solve = zeros(size(rip_clique));

N = 20;
n_loop_closures = 5;
mle = false;
clean = true;
% If double_cover == true, DON'T use q_xi > 0
double_cover = true;

%% Run Experiments
% Need to look for correlation between RIP fixing algorithm failing and the
% solver not returning a valid solution (high rank) - this seemed to be the
% case at least once!
for idx=1:n_runs
    %% Setup poses
    Rs = zeros(3,3,N);
    qs = zeros(4*N, 1);
    Rs(:,:,1) = [1 0 0; 0 0 -1; 0 1 0];  %eye(3);
    qs(1:4) = bot_matrix_to_quat(Rs(:,:,1)); %[0; 1; 0; 0];
    ang_max = pi/12;
    for jdx=2:N
        Rs(:,:,jdx) = Rs(:,:,jdx-1)*random_rot_matrix(3, ang_max);
        qsi = bot_matrix_to_quat(Rs(:,:,jdx));
        qs(4*(jdx-1)+1:4*(jdx-1)+4) = qsi;
    end
    qs_mat = reshape(qs, 4, []);

    %% Add edges
    n_loop_closures = 3;
    E = make_random_pose_graph(N, n_loop_closures);
    M = size(E,1);
    w = ones(M, 1);
    q_meas = zeros(4, M);
    R_meas = zeros(3,3,M);
    %% Add noise
    ang_noise_max = pi/4;% pi/192;
    for jdx=1:M
        R1 = Rs(:,:,E(jdx,1));
        R2 = Rs(:,:,E(jdx,2));
        dR = R1.'*R2*random_rot_matrix(3, ang_noise_max);
        qmi = bot_matrix_to_quat(dR);
        R_meas(:,:,jdx) = dR;
        q_meas(:,jdx) = qmi;
    end
    [sol, psol, pop] = rotation_averaging_sbsos(q_meas, R_meas, ... 
                    E, qs(1:4), clean, w, double_cover, mle);         
    if psol.rnk == 1
        is_solved(idx) = 1;
    end
    if verify_rip(pop.I_clique)
        rip_clique(idx) = 1;
    end
    if verify_rip(pop.I)
        rip_fast(idx) = 1;
    end
    t_solve(idx) = sol.tSOL;
end