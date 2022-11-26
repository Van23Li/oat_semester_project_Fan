function [closestNode, closestNode_int, M, I] = nearest(Dist,sample, RRTree, cfg)
% [I,M] = knnsearch(RRTree(1:2,:)',sample','Distance','cityblock');
% closestNode = RRTree(1:cfg.dim, I);
% closestNode_int = RRTree(end, I);


distances = Dist(sample, RRTree(1:cfg.dim, :));
% Find the closest node.
[M, I] = min(distances, [], 2);
closestNode = RRTree(1:cfg.dim, I);
closestNode_int = RRTree(end, I);
end