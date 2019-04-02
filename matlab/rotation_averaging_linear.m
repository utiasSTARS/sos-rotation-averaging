function [qs] = rotation_averaging_linear(E, q_meas, w, q0)
%rotation_averaging_linear 
N = max(max(E));
M = size(q_meas, 2);

if nargin < 3
    w = ones(1, M);
end

if nargin < 4
    q0 = [1; 0; 0; 0];
end

%% Form the matrix in the quadratic cost minimization
% Q = zeros(4*N, 4*N);
% for m=1:M
%     idx = E(m, 1);
%     jdx = E(m, 2);
%     rows_j = 4*jdx-3:4*jdx;
%     cols_i = 4*idx-3:4*idx;
%     Q(rows_j, cols_i) = w(m)^2*quatmultMatrixRight(q_meas(:,m));
% end
% % Solve the eigen-problem
% [V, D] = eig(Q);
% d = diag(D);
% [~, max_ind] = max(real(d));
% % Eigenvector corresponding to the largest eigenvalue
% qs = real(V(:, max_ind));

%% Form the matrix Q in the Q*q = 0 form (noise free)
Q = zeros(4*M, 4*N);
for m=1:M
    idx = E(m,1);
    jdx = E(m,2);
    cols_i = 4*idx-3:4*idx;
    cols_j = 4*jdx-3:4*jdx;
    rows_ind = 4*m-3:4*m;
    Q(rows_ind, cols_i) = w(m)^2*quatmultMatrixRight(q_meas(:,m));
    Q(rows_ind, cols_j) = -eye(4);
end

% Get the column corresponding to the smallest singular value 
[~,~,V] = svd(Q);
qs = V(:,end);

%% Normalize this solution and make the first quaternion q0
qs(1:4) = qs(1:4)/norm(qs(1:4));
Q_shift = quatmultMatrixLeft(quatmultMatrixRight(qs(1:4))\q0);
qs(1:4) = Q_shift*qs(1:4);
for n=2:N
    qs(4*n-3:4*n) = Q_shift*(qs(4*n-3:4*n)/norm(qs(4*n-3:4*n)));
end

end

