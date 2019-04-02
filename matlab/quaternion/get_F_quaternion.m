function [F] = get_F_quaternion(q_meas, E, w)
N = max(max(E));
M = size(q_meas, 2);
assert(M == size(E,1));
if nargin < 3
    w = ones(M, 1);
end
% There are 36 monomials per term in the cost function
% There are 4*N variables plus 1 term for the coefficient
F = sparse(36*M, 4*N + 1);
for m=1:M
    Qij = quatmultMatrixRight(q_meas(:,m));
    %A = [Qij -eye(4)].'*[Qij -eye(4)];
    A = [eye(4) -Qij.'; -Qij eye(4)];
    ind1 = E(m,1);
    ind2 = E(m,2);
    F_ind = 1;
    
    for idx=1:8
        % First 4 idx map to the first quaternion given by E(m,1)
        % k_idx indexes quaternion components in input vector stacking 
        % the quaternions vertically (column vector input)
        if idx <= 4
            k_idx = 4*(ind1-1) + idx;
        else
            k_idx = 4*(ind2-1) + idx - 4;
        end
        for jdx=1:idx
            if jdx <= 4
                k_jdx = 4*(ind1-1) + jdx;                
            else
                k_jdx = 4*(ind2-1) + jdx-4;
            end
            if idx == jdx
                F(36*(m-1) + F_ind, k_idx) = 2;
                F(36*(m-1) + F_ind, end) = w(m)^2*A(idx, jdx);
            else
                F(36*(m-1) + F_ind, k_idx) = 1;
                F(36*(m-1) + F_ind, k_jdx) = 1;
                F(36*(m-1) + F_ind, end) = w(m)^2*2*A(idx, jdx);
            end
            F_ind = F_ind + 1;
        end
    end
end

end