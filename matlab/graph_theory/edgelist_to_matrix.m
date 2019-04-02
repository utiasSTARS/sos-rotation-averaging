function [A] = edgelist_to_matrix(E)
%edgelist_to_matrix 
M = size(E,1);
N = max(max(E));
A = zeros(N,N);

for idx=1:M
    i1 = min(E(idx,:));
    i2 = max(E(idx,:));
    A(i1, i2) = 1;
end

end

