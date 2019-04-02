%% results_rotation_averaging_sbsos_odom.m
clear; clear all; close all; clc;
eq_tol = 1e-8;
%% Setup random data - no noise
% rand('seed', 1);
rand('seed', 663487230522346);
%% SETUP AN EVEN BIGGER LOOP
%N_array = [20, 40, 80, 120, 160, 200]; % number of poses
N_array = [50];%[80, 100];
n_noise = 1; %10;% discretization of noise
noise_max_ang = pi/9; %pi  %max magnitude of noise
n_problems = 10; % number of problems
mle = false; 
clean = true; % parameter for RIP algorithm
double_cover = true;
%ang_max = pi/12; % max angle difference between poses
qs_init = [1; 0; 0; 0]; % initial quaternion constant

for ndx = 1: size(N_array,2) % loop overnumber of poses
    N = N_array(ndx); % number of poses
%     n_closures_array = [1, N/20, N/10, N/5, N/2]; % generate n closures
    n_closures_array = [0]; %[5];%[1, 2, 3, 4]; % generate n closures

    for cdx = 1: size(n_closures_array,2) % loop over number of closures
        
        % Intiialize results for the SBSOS solver
        f_results_sbsos = zeros(n_noise, n_problems);
        f_results_check_sbsos = zeros(size(f_results_sbsos));
        obj_results_sbsos = zeros(size(f_results_sbsos));
        succeed_results_sbsos = zeros(size(f_results_sbsos));
        t_results_sbsos = zeros(size(f_results_sbsos));
        error_results_sbsos = zeros(size(f_results_sbsos));
        error_norm_sbsos = zeros(size(f_results_sbsos));
        error_quat_sbsos = zeros(size(f_results_sbsos));
        error_norm_quat_sbsos = zeros(size(f_results_sbsos));
        rip_satisfied = zeros(size(f_results_sbsos));
        
        % Initialize results for the dual solver
        error_results_dual = zeros(size(f_results_sbsos));
        f_results_dual = zeros(size(f_results_sbsos));
        obj_results_dual = zeros(size(f_results_sbsos));
        error_quat_dual = zeros(size(f_results_sbsos));
        error_norm_dual = zeros(size(f_results_sbsos));
        error_norm_quat_dual = zeros(size(f_results_sbsos));
        t_results_dual = zeros(size(f_results_sbsos));
        
        % Initialize results for the normalized linear solution
        error_results_linear = zeros(size(f_results_sbsos));
        f_results_linear = zeros(size(f_results_sbsos));
        t_results_linear = zeros(size(f_results_sbsos));
        error_norm_linear = zeros(size(f_results_sbsos));
        error_norm_quat_linear = zeros(size(f_results_sbsos));
        error_quat_linear = zeros(size(f_results_sbsos));
        
        % Initialize results for the local solution (fmincon)
        error_results_local = zeros(size(f_results_sbsos));
        f_results_local = zeros(size(f_results_sbsos));
        t_results_local = zeros(size(f_results_sbsos));
        error_quat_local = zeros(size(f_results_sbsos));
        error_norm_local = zeros(size(f_results_sbsos));
        error_norm_quat_local = zeros(size(f_results_sbsos));
        
        % Initialize results for the local solution from odometry init. (fmincon)
        error_results_odom = zeros(size(f_results_sbsos));
        f_results_odom = zeros(size(f_results_sbsos));
        t_results_odom = zeros(size(f_results_sbsos));
        error_quat_odom = zeros(size(f_results_sbsos));
        error_norm_odom = zeros(size(f_results_sbsos));
        error_norm_quat_odom = zeros(size(f_results_sbsos));
        
        n_loop_closures = n_closures_array(cdx);
        
        for kdx=1:n_problems % Each (N, n_loop_closures) gets n random problems
            
            Rs = zeros(3,3,N);
            qs = zeros(4*N, 1);
            Rs(:,:,1) = eye(3);
            qs(1:4) = qs_init;
            for idx=2:N %2:N
                Rs(:,:,idx) = Rs(:,:,idx-1)*random_rot_matrix(3);
                qsi = bot_matrix_to_quat(Rs(:,:,idx));
                % Only use positive w half of the double cover to start
                if qsi(1) < 0
                    qsi = -qsi;
                end
                qs(4*(idx-1)+1:4*(idx-1)+4) = qsi;
            end
            qs_mat = reshape(qs, 4, []);
            
            %% Add edges
            E = make_random_pose_graph(N, n_loop_closures);
            M = size(E,1);
            
            ang_noise_max_vec = linspace(0.0, noise_max_ang, n_noise);
            
            for jdx=1:n_noise
                
                q_meas = zeros(4, M);
                R_meas = zeros(3,3,M);
                ang_noise_max = ang_noise_max_vec(jdx);
                
                for idx=1:M
                    R1 = Rs(:,:,E(idx,1));
                    R2 = Rs(:,:,E(idx,2));
                    dR = R1.'*R2*random_rot_matrix(3, ang_noise_max);
                    qmi = bot_matrix_to_quat(dR);
                    if qmi(1) < 0
                        qmi = -qmi;
                    end
                    R_meas(:,:,idx) = dR;
                    q_meas(:,idx) = qmi;
                end
                
                mst = E(1:N-1,:);
                [q_meas, qs_odom] = set_measurements_with_mst(E, mst, q_meas, qs_init, 1);
                w = ones(M, 1);
%                 w = rand(M, 1)*3.0 + 0.1;
                
                % Use normalized linear eigen-solution
                tic;
                [q_sol_lin] = rotation_averaging_linear(E, q_meas, w);
                t_results_linear(jdx,kdx) = toc;
                f_results_linear(jdx,kdx) = f_quaternion_loss(q_meas, E, ...
                    reshape(q_sol_lin, 4, []), w);
                [e, e_norm] = get_abs_error(q_sol_lin, qs);
                error_results_linear(jdx,kdx) = e;
                error_norm_linear(jdx,kdx) = e_norm;
                [e, e_norm] = get_abs_error_quat(q_sol_lin, qs);
                error_quat_linear(jdx,kdx) = e;
                error_norm_quat_linear(jdx,kdx) = e_norm;
                
                % Use local search (fmincon) to refine
                tic;
                [q_sol_local] = rotation_averaging_local(E, q_meas, q_sol_lin, w);
                t_results_local(jdx,kdx) = toc;
                f_results_local(jdx,kdx) = f_quaternion_loss(q_meas, E, ...
                    reshape(q_sol_local, 4, []), w);
                [e, e_norm] = get_abs_error(q_sol_local, qs);
                error_results_local(jdx,kdx) = e;
                error_norm_local(jdx,kdx) = e_norm;
                [e, e_norm] = get_abs_error_quat(q_sol_local, qs);
                error_quat_local(jdx,kdx) = e;
                error_norm_quat_local(jdx,kdx) = e_norm;
                
                % Use local search (fmincon) on odometry (MST open loop)
                tic;
                [q_sol_odom] = rotation_averaging_local(E, q_meas, ... 
                                            reshape(qs_odom, [], 1), w);
                t_results_odom(jdx,kdx) = toc;
                f_results_odom(jdx,kdx) = f_quaternion_loss(q_meas, E, ...
                    reshape(q_sol_odom, 4, []), w);
                [e, e_norm] = get_abs_error(q_sol_odom, qs);
                error_results_odom(jdx,kdx) = e;
                error_norm_odom(jdx,kdx) = e_norm;
                [e, e_norm] = get_abs_error_quat(q_sol_odom, qs);
                error_quat_odom(jdx,kdx) = e;
                error_norm_quat_odom(jdx,kdx) = e_norm;
                
                % Use SBSOS solver
                [sol, psol, pop, info] = rotation_averaging_sbsos(q_meas, R_meas, E, ...
                    qs(1:4), clean, w, double_cover, mle); % sbsos solver
                if isfield(psol, 'xsol')
                    succeed_results_sbsos(jdx, kdx) = true;
                    f_results_check_sbsos(jdx, kdx) = ...
                        f_quaternion_loss(q_meas, E, ...
                        reshape(psol.xsol, 4, []), w);
                    f_results_sbsos(jdx, kdx) = psol.fx;
                    [e, e_norm] = get_abs_error(psol.xsol, qs);
                    error_results_sbsos(jdx,kdx) = e;
                    error_norm_sbsos(jdx,kdx) = e_norm;
                    [e, e_norm] = get_abs_error_quat(psol.xsol, qs);
                    error_quat_sbsos(jdx,kdx) = e;
                    error_norm_quat_sbsos(jdx,kdx) = e_norm;
                    
                    % COMPARE 
%                     q_meas2 = q_meas;
%                     q_meas2(:,end) = -q_meas(:,end); 
%                     [sol2, psol2, pop2, info2] = rotation_averaging_sbsos(q_meas2, R_meas, E, ...
%                     qs(1:4), clean, w, double_cover, mle);
%                     [e2, e_norm2] = get_abs_error_quat(psol2.xsol, qs);
%                     e_norm
%                     e_norm2
                
                else
                    succeed_results_sbsos(jdx, kdx) = false;
                    f_results_sbsos(jdx, kdx) = NaN;
                    error_results_sbsos(jdx,kdx) = NaN;
                    error_norm_sbsos(jdx,kdx) = NaN;
                    error_quat_sbsos(jdx,kdx) = NaN;
                    error_norm_quat_sbsos(jdx,kdx) = NaN;
                end
                t_results_sbsos(jdx,kdx) = psol.ttot;
                obj_results_sbsos(jdx, kdx) = psol.obj;
                rip_satisfied(jdx,kdx) = verify_rip(pop.I);
                blkdim = info{1}; % see lines 182-194 in sdpt3.m
                numblk = info{2};
                
                n_sdp_variables_sbsos(jdx,kdx) = blkdim(1); % not sure if this varies with different problems
                n_sdp_blk_sbsos(jdx,kdx) = numblk(1);
                n_linear_var_sbsos(jdx,kdx) = blkdim(3);
                n_free_var_sbsos(jdx,kdx) = blkdim(4);
                n_constr_sbsos(jdx,kdx) = info{3};
                
                
                % Use Lagrangian Dual Solver
%                 tic;
%                 [q_sol_dual, dual_obj] = duality_solver(q_meas, E, qs_init, w);
%                 q_sol_dual = [qs_init; q_sol_dual];
%                 t_results_dual(jdx,kdx) = toc;
%                 f_results_dual(jdx,kdx) = f_quaternion_loss(q_meas, E, ...
%                     reshape(q_sol_dual, 4, []), w);
%                 error_results_dual(jdx,kdx) = get_abs_error(q_sol_dual, qs);
%                 error_quat_dual(jdx,kdx) = get_abs_error_quat(q_sol_dual, qs);
%                 obj_results_dual(jdx,kdx) = dual_obj;
%                 
%                 n_sdp_variables_dual(jdx,kdx) = (N*4 + 1); % do every iteration for consistency with sbsos
%                 n_constr_dual(jdx,kdx) = (N-1) + 2; % all non-anchor quats + (Z(end,end) = 1) + posdef
%                 
                
            end
        end
        
        min(min(t_results_sbsos))
        mean(mean(t_results_sbsos))
        max(max(t_results_sbsos))
        min(min(t_results_dual))
        mean(mean(t_results_dual))
        results_string = ['../data/results/results_odom/results_sbsos_loop_' num2str(n_loop_closures) '_N_' num2str(N) ...
            '_double_cover_' num2str(double_cover) '_noise_max_' ...
            num2str(ang_noise_max) '_runs_per_noise_' ...
            num2str(n_problems) '.mat'];
%         save(results_string);
    end
end

