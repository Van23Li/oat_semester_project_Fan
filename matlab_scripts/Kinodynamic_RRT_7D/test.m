clc
close all force
clear all

cfg.dim = 2
pos_enc = @(x)[x sin(x) cos(x)];

%gmm model creation
if cfg.dim == 2
    load('data/net50_pos_thr.mat')
    [cfg.y_f, cfg.dy_f] = tanhNN(net);
elseif cfg.dim == 7
    load('data_7d/net128_pos.mat')
    [cfg.y_f, cfg.dy_f] = tanhNN(net);
end

inp = pos_enc([0,0,0,2])';
val = cfg.y_f(inp);
dval = cfg.dy_f(inp)