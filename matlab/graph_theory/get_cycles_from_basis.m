function [C] = get_cycles_from_basis(b)
%get_cycles_from_basis 

C = {}; 
for idx=1:size(b,2)
    C{idx} = find(b(:,idx)).';
end

% Now do pairwise differences (symmetric differences)

for idx=1:size(b,2)
    for jdx=idx+1:size(b,2)
        C{end+1} = find(xor(b(:,idx), b(:,jdx))).';
    end    
end

end

