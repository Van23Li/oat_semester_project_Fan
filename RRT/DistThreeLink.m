function [out, I] = DistThreeLink (x1, x2)
% Compute the distance between two sets of three link coordinates
% Note we assume all angular coordinates are between 0 and 360
% 最后一行标志是正向还是反向
e = abs(bsxfun(@minus, x2, x1));
out = sum(min(e, 360-e));
I = (e<360-e)*2-1;
