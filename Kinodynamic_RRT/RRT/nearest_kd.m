function [closestNode, closestNode_int, M, I, T] = nearest_kd(Dist_kd, sample, RRTree, cfg)

%% Find the closest node.
distance = Dist_kd(sample, RRTree, cfg);
[M,I] = min(distance);
closestNode = RRTree(1:2*cfg.dim, I);
closestNode_int = RRTree(end, I);
T = distance(:,I);
    if sum(T) < 0
        a=2
    end
end