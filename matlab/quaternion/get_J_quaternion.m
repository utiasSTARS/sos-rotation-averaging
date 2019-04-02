function [J] = get_J_quaternion(I, N, mle)
%get_J_quaternion Gives a cell array J containing an array for each element
%of I that lists the constraints in G that correspond to that element.

if nargin < 3
    mle = false;
end

n = length(I);
J = cell(1, n);
for idx=1:n
    Ii = I{idx};
    % To start, add the 'special' constraints
    if any(ismember([1 2 3 4], Ii))
        if mle
            Ji = [3*N+8+idx 3*N+1:3*N+8];
        else
            Ji = [3*N+1:3*N+8];
        end
    else
        if mle
            Ji = [3*N+8+idx];
        else
            Ji = [];
        end
    end
    for jdx=Ii
        j_Ii_jdx = ceil(jdx/4);
        Ji = [Ji 3*j_Ii_jdx-2 3*j_Ii_jdx-1 3*j_Ii_jdx];
    end
    J{idx} = sort(unique(Ji));
end

end
