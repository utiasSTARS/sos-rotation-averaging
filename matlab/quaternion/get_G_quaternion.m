function [G] = get_G_quaternion(N, q0, I)
%get_G_quaternion N is the number of quaternion variables

if nargin < 2
    q0 = bot_matrix_to_quat([1 0 0; 0 0 -1; 0 1 0]);
end

G = cell(3*N + 8, 1);
for n=1:N
    % Positive x element of quat constraint
    Gn1 = sparse(1, 4*N+1);
    Gn1(4*(n-1) + 2) = 1;
    Gn1(end) = 1;
    G{3*(n-1) + 1} = Gn1;
    
    Gn2 = sparse(5, 4*N+1);
    Gn2(1, 4*(n-1) + 1) = 2;
    Gn2(1, end) = -1;
    Gn2(2, 4*(n-1) + 2) = 2;
    Gn2(2, end) = -1;
    Gn2(3, 4*(n-1) + 3) = 2;
    Gn2(3, end) = -1;
    Gn2(4, 4*(n-1) + 4) = 2;
    Gn2(4, end) = -1;
    Gn2(5, end) = 1;
    G{3*(n-1) + 2} = Gn2;
    
    Gn3 = sparse(5, 4*N+1);
    Gn3(1, 4*(n-1) + 1) = 2;
    Gn3(1, end) = -1;
    Gn3(2, 4*(n-1) + 2) = 2;
    Gn3(2, end) = -1;
    Gn3(3, 4*(n-1) + 3) = 2;
    Gn3(3, end) = -1;
    Gn3(4, 4*(n-1) + 4) = 2;
    Gn3(4, end) = -1;
    Gn3(5, end) = 2;
    G{3*(n-1) + 3} = Gn3;
end

% Finally, fix the first variable to q0 = [0 1 0 0] by default. 
% [1 0 0 0].' was giving trouble because right on the qx >= 0 boundary. 
%q2 = 1

G_eq = quaternion_equality_constraint(q0,N,1);
assert(length(G_eq) == 8);
for idx=1:8
    G{3*N+idx} = G_eq{idx};
end

if nargin >= 3
    G_quartic = get_G_quartic(N, I);
    G = [G; G_quartic];
end

end