function [G] = get_G_quaternion_double_cover(N, q0, I)
%get_G_quaternion_double_cover N is the number of quaternion variables

% G is the constraint matrix. For the double quaternion cover, each
% quaternion has 2 constraint equations (left and right bounds because it's
% an equality for unit length), and the first quaternion variable has
% equality constraints. 

G = cell(2*N + 8, 1);
if nargin < 2
    q0 = bot_matrix_to_quat([1 0 0; 0 0 -1; 0 1 0]);
end

for n=1:N
    
    Gn1 = sparse(5, 4*N+1);
    Gn1(1, 4*(n-1) + 1) = 2;
    Gn1(1, end) = -1;
    Gn1(2, 4*(n-1) + 2) = 2;
    Gn1(2, end) = -1;
    Gn1(3, 4*(n-1) + 3) = 2;
    Gn1(3, end) = -1;
    Gn1(4, 4*(n-1) + 4) = 2;
    Gn1(4, end) = -1;
    Gn1(5, end) = 1;
    G{2*(n-1) + 1} = Gn1;
    
    Gn2 = sparse(5, 4*N+1);
    Gn2(1, 4*(n-1) + 1) = 2;
    Gn2(1, end) = -1;
    Gn2(2, 4*(n-1) + 2) = 2;
    Gn2(2, end) = -1;
    Gn2(3, 4*(n-1) + 3) = 2;
    Gn2(3, end) = -1;
    Gn2(4, 4*(n-1) + 4) = 2;
    Gn2(4, end) = -1;
    Gn2(5, end) = 2;
    G{2*(n-1) + 2} = Gn2;
end

% Finally, fix the first variable to q0 by default. 
% [1 0 0 0].' was giving trouble because right on the qx >= 0 boundary. 
% [0 1 0 0].' was also giving trouble because of rot-mat to quat
% difficulties for 180 rotation (which is what that one is about x). So 
% I settled on a 90 degree rotation about x: [sqrt(2) sqrt(2) 0 0].'

G_eq = quaternion_equality_constraint(q0,N,1);
assert(length(G_eq) == 8);
for idx=1:8
    G{2*N+idx} = G_eq{idx};
end

if nargin >= 3
    G_quartic = get_G_quartic(N, I);
    G = [G; G_quartic];
end

end