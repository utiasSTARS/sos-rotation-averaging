function [I] = get_I_quaternion(E, clean)
if nargin < 2
    clean = false;
end
N = max(max(E));
M = size(E, 1);
I = cell(1, M);
I_min = zeros(1, M); 
%% TODO: compare with doing parents like in JT paper for BN
for m=1:M % go through all edges
    
    idx = E(m, 1); % indice of first
    jdx = E(m, 2); % indice of second
    
    Imi = 4*(idx - 1)+1:4*(idx-1)+4; % actual indices since it's quaternions
    Imj = 4*(jdx - 1)+1:4*(jdx-1)+4; 
    I{m} = sort([Imi Imj]); % store indices for m-th edge (sort in ascending order)
    I_min(m) = min(I{m}); % store the smallest indice for this edge
end

%% Sort so that increases by minimal element
[alpha, ind_sort] = sort(I_min); % sort the smallest indices in scending order
I = I(ind_sort); % sort the actual edges in ascending order of their lowest indice

%% Form Junction Tree to satisfy running intersection property (RIP)
I = junction_tree(I, alpha, clean); % alpha is the actual array of sorted minimal elements.

end
