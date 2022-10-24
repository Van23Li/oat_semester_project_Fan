function RRTree = IntermediateStates(RRTree, closestNode, newPoint, I)
% I: closestNode's int



RRTree = [RRTree, [newPoint; RRTree(end-1,I) + T ;I]];

end