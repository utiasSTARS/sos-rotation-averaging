function [eq] = edge_queue(mst, anchor_id)
%edge_queue Lay out the edges of an MST in an order that can be used to
% estimate a trajectory.
    eq = zeros(size(mst));
    eq_idx = 1;
    current_edge_idx = get_next_edge_idx(mst, anchor_id);
    current_vertex = anchor_id;
    completed_vertices = [];
    seen_vertices = [];
    while size(mst, 1) > 0
        current_edge = mst(current_edge_idx, :);
        eq(eq_idx,:) = current_edge;
        eq_idx = eq_idx + 1;
        mst(current_edge_idx, :) = [];
        if ~ismember(current_edge(current_edge ~= current_vertex), ...
                     seen_vertices)
            seen_vertices = ...
              [seen_vertices current_edge(current_edge ~= current_vertex)];
        end
        while ~ismember(current_vertex, mst)
            completed_vertices = [completed_vertices current_vertex];
            if isempty(seen_vertices)
                return
            end
            current_vertex = seen_vertices(1);
            seen_vertices(1) = [];
        end
        current_edge_idx = get_next_edge_idx(mst, current_vertex);
    end
end

function [edge_idx] = get_next_edge_idx(mst, id)
    [I, ~] = find(mst == id);
    if isempty(I)
        edge_idx = [];
        return
    end
%     [~, edge_idx] = min(I);
    edge_idx = min(I);
end