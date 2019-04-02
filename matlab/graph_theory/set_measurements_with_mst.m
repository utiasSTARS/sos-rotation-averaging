function [qm_out, qs] = set_measurements_with_mst(E, mst, qm_in, ... 
                                    q_anchor, anchor_id)
%set_measurements_with_mst 

if nargin < 5
% Assume mst(1,1) is the anchored vertex 
    anchor_id = mst(1,1);
end
N = length(unique(E));
M_mst = size(mst, 1);
assert(M_mst == N-1);
% Keep track of "open loop" odometry
qs = zeros(4, N);
qs(:, anchor_id) = q_anchor;
qm_out = zeros(size(qm_in));
% Construct queue of edges with the MST
eq = edge_queue(mst, anchor_id);
% 'Open loop' odometry for the MST's edges
verts_seen = [anchor_id];
for idx=1:M_mst
    current_edge = eq(idx,:);
    qm_idx = E(:,1) == current_edge(1) & E(:,2) == current_edge(2);
    qm = qm_in(:, qm_idx);
    qm_out(:, qm_idx) = qm_in(:, qm_idx);
    if ismember(current_edge(1), verts_seen)
        qs(:, current_edge(2)) = quatmultMatrixLeft(qs(:, current_edge(1)))*qm;
        verts_seen = [verts_seen current_edge(2)];
    else
        assert(ismember(current_edge(2), verts_seen), 'Error in MST ordering');
        qs(:, current_edge(1)) = quatmultMatrixRight(qm)\qs(:, current_edge(2));
        verts_seen = [verts_seen current_edge(1)];
    end
end
assert(all(sort(verts_seen).' == sort(unique(E))), 'Some vertices not seen.');

% Choose the error minimizing sign for the remaining edges
M = size(E, 1);
for idx=1:M
    if ~ismember(E(idx,:), mst, 'rows')
%         qm_idx = E(:,1) == current_edge(1) & E(:,2) == current_edge(2);
        qm = qm_in(:, idx);
        q1 = qs(:, E(idx,1));
        q2 = qs(:, E(idx,2));
        if norm(quatmultMatrixLeft(q1)*qm - q2) < norm(-quatmultMatrixLeft(q1)*qm - q2)
            qm_out(:, idx) = qm;
        else
            qm_out(:, idx) = -qm;
        end
    end
end


end
