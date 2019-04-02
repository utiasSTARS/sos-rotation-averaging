function [sol, psol, pop, info] = rotation_averaging_sbsos(q_meas, R_meas, E, q0, clean, w, double_cover, mle)
%rotation_averaging_sbsos Rotation averaging with q_meas(:,k) relative
% measurement between variable E(k,1) and E(k,2)

%% Check size of inputs
N = max(max(E));
M = size(q_meas, 2);
assert(M == size(E,1));

%% Parameters
if nargin < 4
    q0 = [1; 0; 0; 0];
end
  
if nargin < 5
    clean = true;
end
if nargin < 6
    w = ones(M, 1);
end
if nargin < 7
    double_cover = false;
end

if nargin < 8
    mle = true;
end

%% Form data matrix F for the polynomial optimization problem cost function
if mle
    F = get_F_mle(R_meas, E, w);
else
    F = get_F_quaternion(q_meas, E, w);
end

%% Form I
% Smail method (Junction Trees) NOT WORKING
% I = get_I_quaternion(E, clean);
I = get_I(E, clean, false);

% TEMPORARY CHECK
% I{1} = 1:4*N;
% I_clique = get_rip_cliques(E);

%% Form data matrices cell array G 
% TODO: may need to add redundant constraints as per Theorem 3 of Weisser,
% Lasserre, Toh 2018.
if double_cover
    if mle
        G = get_G_quaternion_double_cover(N, q0, I);
    else
        G = get_G_quaternion_double_cover(N, q0);
    end
else
    if mle
        G = get_G_quaternion(N, q0, I);
    else
        G = get_G_quaternion(N, q0);
    end
end
%% Form partition matrix J
% %% TEMPORARY TEST
% I = {[5 6 7 8 9 10 11 12 13 14 15 16];
%          [1 2 3 4 5 6 7 8 12 13 14 15 16]};
if double_cover
    J = get_J_quaternion_double_cover(I,N,mle);
else
    J = get_J_quaternion(I,N,mle);
end
    %% Form data matrices list G for the POP constraints
%% Solve with SBSOS
pop.n = N;
pop.F = F;
pop.G = G;
pop.I = I;
% pop.I{1} = 1:16;
pop.J = J;

% pop.I_clique = I_clique;

if mle
    pop.k = 2;
else
    pop.k = 1;
end
pop.d = 1; %1;
sdp = gendata2(pop,'SBSOS');
[sol, info] = csol(sdp, 'sdpt3'); % Try 'sedumi' too

psol = postproc(pop,sdp,sol);
%sdp
%% Extract solution 


end


