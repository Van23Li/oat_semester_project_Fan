clear all

load fisheriris
X = meas;
[n,k] = size(X);

Mdl1 = KDTreeSearcher(X);