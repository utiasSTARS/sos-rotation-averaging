function [f] = f_quaternion_loss(q_meas, E, qs, w)
%f_quaternion_loss 
N = max(max(E));
M = size(E,1);
assert(M == size(q_meas, 2));
if nargin < 4
    w = ones(1, M);
end
f = 0;
for kdx=1:M
    Qij = quatmultMatrixRight(q_meas(:,kdx));
    qi = qs(:, E(kdx,1));
    qj = qs(:, E(kdx,2));
    % Problem at identity (the poles!!)
    f = f + w(kdx)^2*norm(Qij*qi - qj)^2;
end

end

