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

inp = pos_enc([0,0,0,1])';
ObsPoint = [0,0];
val = cfg.y_f(inp)
% val_1_dy = cfg.dy_f(inp)
% a = (val_1_dy(cfg.dim + 1 : cfg.dim + 2)+...
%         val_1_dy(2*cfg.dim + 3 : 2*cfg.dim + 4) .* cos([ObsPoint(1),ObsPoint(2)])+...
%         val_1_dy(3*cfg.dim + 5 : 3*cfg.dim + 6) .* (-sin([ObsPoint(1),ObsPoint(2)])))/3