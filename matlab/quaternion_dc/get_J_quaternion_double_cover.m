function [J] = get_J_quaternion_double_cover(I, N, mle)
%get_J_quaternion_double_cover Gives a cell array J containing an array for each element
%of I that lists the constraints in G that correspond to that element.
% Inputs
% I is the sparsity pattern for the cost function
% N is the number of quaternion variables

% Current state may be VERY slow, very redundant.

if nargin < 3
    mle = false;
end

n = length(I);
J = cell(1, n);
for idx=1:n
    Ii = I{idx};
    if any(ismember([1 2 3 4], Ii))
        if mle
            Ji = [2*N+8+idx 2*N+1:2*N+8];
        else
            Ji = [2*N+1:2*N+8];
        end
    else
        if mle
            Ji = [2*N+8+idx]; % zeros(1, 2*length(Ii));
        else 
            Ji = [];
        end
    end
    for jdx=Ii
        j_Ii_jdx = ceil(jdx/4);
        Ji = [Ji 2*j_Ii_jdx-1 2*j_Ii_jdx];
    end
    J{idx} = sort(unique(Ji));
end

end
