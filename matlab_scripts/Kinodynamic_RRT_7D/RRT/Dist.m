function out = Dist (x1, x2)
% Here we assume that the Dist function can compute the
% distance to multiple samples corresponding to the columns of
% the second argument
% at the end of this call the array distances will indicate the
% distance between the new sample and each of the samples that has been
% generated so far in the program.
    
e = abs(bsxfun(@minus, x2, x1));
out = sum(e);
