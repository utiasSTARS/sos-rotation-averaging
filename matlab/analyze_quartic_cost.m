%% Analyze quaternion element rot matrix representation
clear; close all; clc;
% Need SOSTools and Sedumi on the Matlab path
%% Define variables
qi = sym('qi', [4 1]);
qj = sym('qj', [4 1]);
Rij = sym('Rij', [3 3]);

Ri = quat_to_matrix(qi); % bot_quat_to_matrix(qi);
Rj = quat_to_matrix(qj); %bot_quat_to_matrix(qj);

z = sym('z', [8,1]); % For lemma

%% First quickly check SO(3) constraints
f_orthog = Rij(:,1).'*Rij(:,2);
H_orthog = hessian(f_orthog, Rij(:));
z_orthog = sym('z_o', [9,1]);
f_orthog_z = simplify(z_orthog.'*H_orthog*z_orthog);
[Q1, Z1] = findsos(f_orthog_z)
% findsos causes an error here, but upon inspection of f_orthog_z it's
% clearly not a SOS function.

%% Check quaternionic loss with SWITCHING constraint
b = sym('b');
qij = sym('qij', [4 1]);
z_quat = sym('zq', [9 1]);
arg_quat_norm = b*quatmultMatrixRight(qij)*qi - qj;
f_quat = simplify(sum(arg_quat_norm.^2));
qij_rand = [0; 1; 0; 0]; %rand(4,1);
qij_rand = qij_rand/norm(qij_rand,2);
H_quat = hessian(f_quat, [qi; qj; b]);
f_quat_z = z_quat.'*H_quat*z_quat;
f_quat_z_num = simplify(subs(f_quat_z, qij(:), qij_rand(:)));

[Q_quat, Z_quat] = findsos(f_quat_z_num)

%% Define cost function 

arg_norm = Ri*Rij - Rj;
f = sum(sum(arg_norm.^2));

%% Compute Hessian
H = hessian(f, [qi; qj])

% This will be hard to prove SOS convexity for...
% Can prove numerically for:

% Hsub = subs(H, qi, [1 0 0 0].');
% Hsub = subs(Hsub, qj, [1 0 0 0].');
Hsub = subs(H, Rij(:), [1 0 0 0 1 0 0 0 1].');
% sqrtm(Hsub)

% If we can prove it, use k=2, d=1 in the Sparse-BSOS hierarchy!
% Need to check kinematics, etc.

%% Use Lemma 1 from Mangelson!
fz = simplify(z.'*H*z);

% Now need to check whether fz is SOS-convex - probably can't be done
% symbolically, have to do random sampling and form an SDP for each one.
% Can SOSTools do basic SOS checking??
N_tests = 5;
vartable = [qi; qj; z];
for idx=1:N_tests
    idx
    R_idx = random_rot_matrix(3);
%     R_idx2 = random_rot_matrix(3);
%     R_idx3 = random_rot_matrix(3);
%     R_idx = [0 1 0; 1 0 0; 0 0 -1];
%     R_idx2 = [0 0 1; 1 0 0; 0 1 0];
%     R_idx3 = [1 0 0; 0 0 -1; 0 1 0];
    fz_idx = simplify(subs(fz, Rij(:), R_idx(:)));
%     fz_idx1 = simplify(subs(fz, Rij(:), R_idx(:)));
%     fz_idx2 = simplify(subs(fz, Rij(:), R_idx2(:)));
%     fz_idx3 = simplify(subs(fz, Rij(:), R_idx3(:)));
%     fz_idx = simplify(fz_idx1 + fz_idx2 + fz_idx3);
    % First, initialize the sum of squares program
%     prog = sosprogram(vartable);   % No decision variables.
%     % Next, define the inequality
%     % p(x1,x2) >=  0
%     prog = sosineq(prog,fz_idx);
%     % And call solver
%     solver_opt.solver = 'sedumi';
%     [prog,info] = sossolve(prog,solver_opt);
%     echo off;
    
    % Can also try rational arguments (shouldn't make a difference):
    % [Q, Z] = findsos(fz_idx, 'rational');
    [Q, Z] = findsos(fz_idx)
    %% RETURNED EMPTY IN ALL CASES, NOT SOS-CONVEX BOOOOOO!!!!
    %% Also saw a problem with nchoosek, think it was path issue??
end

