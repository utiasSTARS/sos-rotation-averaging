function [C_out] = get_rip_cliques(E, n_vars)
%get_rip_cliques 
if nargin < 2
    n_vars = 4;
end
chordal = false;
while ~chordal
    %E_new = [];
    chord_added = false;
    cycle_basis = grCycleBasis(E);
    cycles = get_cycles_from_basis(cycle_basis);
    cycles_chordal = zeros(length(cycles), 1);
    for idx=1:length(cycles)
        ci = cycles{idx};
        if length(ci) < 4
            cycles_chordal(idx) = 1;
            continue
        end
        Eidx = E(ci,:);
        vs_idx = unique(Eidx);
        
        % If cycle is not chordal, make it chordal!
        for jdx=1:size(E,1)
            Ejdx = E(jdx,:);
            if ismember(Ejdx, Eidx, 'rows') || ismember(fliplr(Ejdx), Eidx, 'rows')
                continue 
            end
            if ismember(Ejdx(1), vs_idx) && ismember(Ejdx(2), vs_idx)
                cycles_chordal(idx) = 1;
                break;
            end
        end
        
        % Make the graph chordal if needed
        if ~cycles_chordal(idx)
            for jdx=1:length(vs_idx)
                vj = vs_idx(jdx);
                for kdx=jdx+1:length(vs_idx)
                    vk = vs_idx(kdx);
                    if ~ismember([vj vk], Eidx, 'rows') && ~ismember([vk vj], Eidx, 'rows')
                        %E_new = [E_new; vj vk];
                        E = [E; vj vk];
                        chord_added = true;
                        break;
                    end
                end
                if chord_added
                    break;
                end
            end           
        end   
        if chord_added
            break;
        end
    end
    chordal = all(cycles_chordal);
end




% Get maximal cliques
C = maximalCliques(edgelist_to_matrix(E));
C_min = zeros(1, length(C));
% Sort for RIP! Descending? 
for idx=1:length(C)
    C_min(idx) = min(C{idx});
end
[~, ind_sort] = sort(C_min, 'ascend');
C = C(ind_sort);

%% Expand for quaternions
C_out = cell(length(C), 1);
for idx=1:length(C)
    Cidx = [];
    vals = unique(C{idx});
    
    for val=vals
        Cidx = [Cidx n_vars*(val-1)+1:n_vars*(val-1)+n_vars];
    end
    
    Cidx = sort(unique(Cidx));
    C_out{idx} = Cidx;
end

end

