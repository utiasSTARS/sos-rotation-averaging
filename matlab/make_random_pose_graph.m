function [E] = make_random_pose_graph(N, n_loop_closures)
%make_random_pose_graph Make a random pose graph with odometry "backbone"
%and each pose only appearing in at most one of n_loop_closures.
E = [(1:N-1).' (2:N).'];
% Add some loops!
cands = randperm(N);
idx = 0;
while idx < n_loop_closures
    if abs(cands(1) - cands(2)) > 1.1
        E = [E; cands(1) cands(2)];
        cands = cands(3:end);
        idx = idx + 1;
    else
        inds = randperm(length(cands));
        cands = cands(inds);
    end
end
end

