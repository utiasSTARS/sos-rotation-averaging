function [qs] = rotation_averaging_local(E, q_meas, qs0, w)
%rotation_averaging_local Use fmincon to solve rotation averaging locally

M = size(E,1);
N = max(max(E));
if nargin < 4
    w = ones(1,M);
end

A = zeros(4*M,4*(N-1));
b = zeros(4*M,1);

q_init = qs0(1:4);

%% Generate A, b, and objective function
for m = 1:M % cycle through edges
    Qij = quatmultMatrixRight(q_meas(:,m));
    ind_q_i = E(m,1);
    ind_q_j = E(m,2);
    ind_A_m = 4*m - 3 : 4*m; % row in A matrix
    ind_A_i = 4*ind_q_i - 7 : 4*ind_q_i - 4; % columns corresponding to qi
    ind_A_j = 4*ind_q_j - 7 : 4*ind_q_j - 4; % columns corrsponding to qj
    
    if ind_q_i ~= 1 && ind_q_j ~= 1
        A(ind_A_m, ind_A_i) = Qij*w(m);
        A(ind_A_m, ind_A_j) = -eye(4)*w(m);
    else
        ind_b_m = ind_A_m;
        if ind_q_i == 1 % if we have measurements starting from the initial state we dump that in the b matrix
            b(ind_b_m) = -Qij*w(m)*q_init;
            A(ind_A_m, ind_A_j) = -eye(4)*w(m);
        else  % if measurement closing on anchor
            b(ind_b_m) = q_init*w(m);
            A(ind_A_m, ind_A_i) = Qij*w(m);
        end
    end
end

obj_fun = @(x) (x.'*A.')*A*x -2*x.'*A.'*b;
options = optimoptions('fmincon');
% options.MaxFunctionEvaluations = 30000;
% options.MaxIterations = 5000;
% options.ConstraintTolerance = 1e-6;
qs = fmincon(obj_fun, qs0(5:end), [], [], [], [], -1, 1, ...
             @unit_norm_constraint, options);
%          
% assert(test_unit_quaternions(qs, options.ConstraintTolerance), ...
%                              'Not unit quaternions!');
         
qs = [q_init; qs];

for n=2:N
    qs(4*n-3:4*n) = qs(4*n-3:4*n)/norm(qs(4*n-3:4*n));
end

end

