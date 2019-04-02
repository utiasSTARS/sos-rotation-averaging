function [f] = mle_loss(R_meas, E, Rs, w)
%mle_loss 
N = max(max(E));
M = size(E,1);
assert(M == size(R_meas, 3));
if nargin < 4
    w = ones(1, M);
end
f = 0;
for kdx=1:M
    idx = E(kdx,1);
    jdx = E(kdx,2);
    % Problem at identity (the poles!!)
    f = f + w(kdx)*norm(Rs(:,:,idx)*R_meas(:,:,kdx) - Rs(:,:,jdx), 'fro')^2;
end

end
