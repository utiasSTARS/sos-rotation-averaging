function [L] = graph_laplacian(E, w)
%graph_laplacian Edges are mx2 matrix, weights are vector of m weights
V = unique(E);
m = size(E,1);
n = length(V);
L = zeros(n,n);

if nargin < 2
    w = ones(1, m);
else
    assert(length(w) == m);
end

for idx=1:size(E,1)
    i1 = E(idx,1);
    i2 = E(idx,2);
    L(i1,i2) = -w(idx);
    L(i2,i1) = -w(idx);
    L(i1,i1) = L(i1,i1) + 1;
    L(i2,i2) = L(i2,i2) + 1;
end

end

