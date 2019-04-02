function [G] = quaternion_equality_constraint(q,N,ind)
%quaternion_equality_constraint 
if nargin < 3
    ind = 1;
end
assert(length(q) == 4);
G = cell(8, 1); %zeros(8, 4*N+1);
for idx=1:4
    [g1, g2] = scalar_equality_constraint(q(idx), N, ind+idx-1);
    G{2*(idx-1)+1} = g1;
    G{2*(idx-1)+2} = g2;
end

end

function [g1, g2] = scalar_equality_constraint(v, N, ind)

if v == 0
    g1 = sparse(1, 4*N+1);
else
    g1 = sparse(2, 4*N+1);
    g1(2, end) = v;
end
g1(1, ind) = 1;
g1(1, end) = -1;
if (v + 1) == 0
    g2 = sparse(1, 4*N+1);
else
    g2 = sparse(2, 4*N+1);
    g2(2, end) = v+1;
end
g2(1, ind) = 1;
g2(1, end) = -1;

end