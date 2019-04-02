%% analyze_quartic_mle.m
clear;  close all; clc;

%% Params
qi = sym('qi', [4 1]);
qj = sym('qj', [4 1]);
Rij = sym('Rij', [3 3]);

Ri = quat_to_matrix(qi);
Rj = quat_to_matrix(qj);

arg_norm = Ri*Rij - Rj;
f = simplify(sum(sum(arg_norm.^2)));

[coeffs_i_j, t] = coeffs(f, [qi; qj]);
[coeffs_i_j.' t.']


