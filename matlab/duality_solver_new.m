function [q_sol, d_obj] = duality_solver_new(q_meas, E, q_init)
%DUALITY_SOLVER Solves the rotation averaging problem using "dual of dual"
% method found in Fredriksson, Johan, and Carl Olsson. 
% "Simultaneous multiple rotation averaging using lagrangian duality." , 2012.
% Author: Filip Maric, 2019.

N = max(max(E));
M = size(q_meas, 2);
assert(M == size(E,1));

A = zeros(4*M,4*(N-1));
b = zeros(4*M,1);

%% Generate A, b
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
            b(ind_b_m) = -Qij*q_init;
            A(ind_A_m, ind_A_j) = -eye(4);
        else  % if measurement closing on anchor
            b(ind_b_m) = q_init;
            A(ind_A_m, ind_A_i) = Qij;
        end
    end
end

A_big = [A'*A, - A'*b; -b'*A, b'*b];

%% Solve dual dual
cvx_begin sdp
    variable Z((N-1)*4 + 1,(N-1)*4 + 1) symmetric
    minimize trace(A_big*Z)
    for ind = 1:N-1
        Ni_big = zeros(4*N - 3, 4*N - 3);
        Ni_big(4*ind - 3 : 4*ind, 4*ind - 3 : 4*ind) = eye(4);
        trace(Ni_big*Z) == 1
    end
    Z(end,end) == 1
    Z >= 0
cvx_end

% disp(Z)

q_sol = sqrt(diag(Z(1:4*(N-1), 1:4*(N-1)))); % Z = q*q'
d_obj = trace(A_big*Z);

%disp(vecnorm(q_sol_mat,2))

%% Solve dual of the dual
cvx_begin sdp
    variable Z((N-1)*4, (N-1)*4) symmetric
    minimize(trace


end

