clc
close all force
clear all

addpath('RRT');
addpath('RRT/tradtion_check');
addpath('data');
addpath('data_7d');

% parameters for control
cfg.dim = 2;    % planning in cfg.dim-D case.
cfg.kinodynamic = true;
cfg.circle_obs = true;  % when equals to true, assuming the obstacle as a
% sphere;  when equals to false, dividing it to many points.
cfg.display1 = false;
cfg.display2 = true;
cfg.display3 = true;   % whether to plot figure 1, 2, 3 during planning.
cfg.display4 = false;    % whether to plot figure after planning.
cfg.display5 = true;    % whether to plot q-q_dot.
cfg.maxSample = 2000;  % the maximum number of random samples.

cfg.stepsize = 0.1 * pi;    % 0.095 * pi;
cfg.disTh = 0.1 * pi;
cfg.maxFailedAttempts = 10000;
cfg.RadiusForNeib = 0.2 * pi; % 0.5;    % set the parament (search radius) of RRT*
cfg.saveRand = 0;

pos_enc = @(x)[x sin(x) cos(x)];

%gmm model creation
if cfg.dim == 2
    load('data/net50_pos_thr.mat')
    [cfg.y_f, cfg.dy_f] = tanhNN(net);
elseif cfg.dim == 7
    load('data_7d/net128_pos.mat')
    [cfg.y_f, cfg.dy_f] = tanhNN(net);
end

%cfgeters definition
if cfg.dim == 2
    % 2D robot
    cfg.r = [4 4 0];
    cfg.d = [0 0 0];
    cfg.alpha = [0 0 0];
    cfg.base = eye(4);
%     cfg.q_min = [-2.5, -2.5, -10, -10];
%     cfg.q_max = [2.5, 2.5, 10, 10];
    k1 = 0.95; k2 = 0.95;
    cfg.q_min = [-k1*pi, -k2*pi, -10, -10];
    cfg.q_max = [k1*pi, k2*pi, 10, 10];
    
    cfg.v_min = [-2.0, -2.0];
    cfg.v_max = [2.0, 2.0];
    cfg.a_min = [-10, -10];
    cfg.a_max = [10, 10];
    
elseif cfg.dim == 7
    % 7D robot
    cfg.r = [repmat(1,[1, cfg.dim]), 0];
    cfg.d = 0 * cfg.r;
    cfg.alpha = 0 * cfg.r;
    cfg.base = eye(4);
    k_lim = 0.9;
    cfg.q_min = [repmat(-k_lim * pi, [1,7]), -11, -11];
    cfg.q_max = [repmat(k_lim * pi, [1,7]), 11, 11];
    cfg.q_min(1) = -1.1 * pi;
    cfg.q_max(1) = 1.1 * pi;
end

% Add obstacles
[obs_data] = xlsread('obs_data.xlsx');
if cfg.circle_obs
    obs.centers = obs_data(:,1:2);
    obs.radiuses = obs_data(:,3);
end

% Add start and ending pose
collided1 = 1;
collided2 = 1;
while collided1 || collided2
    if cfg.dim == 2
        %         cfg.start_coords = cfg.q_min(1:cfg.dim)' + (cfg.q_max(1:cfg.dim)-cfg.q_min(1:cfg.dim))'.*rand(cfg.dim,1);
        %         cfg.end_coords = cfg.q_min(1:cfg.dim)' + (cfg.q_max(1:cfg.dim)-cfg.q_min(1:cfg.dim))'.*rand(cfg.dim,1);
        
        cfg.start_coords = [-2.5; -1; 0; 0];  % [q1; q2; v1; v2]
        cfg.end_coords = [2.7; 0; 1; -1];
    elseif cfg.dim == 7
        %         cfg.start_coords = cfg.q_min(1:cfg.dim)' + (cfg.q_max(1:cfg.dim)-cfg.q_min(1:cfg.dim))'.*rand(cfg.dim,1);
        %         cfg.end_coords = cfg.q_min(1:cfg.dim)' + (cfg.q_max(1:cfg.dim)-cfg.q_min(1:cfg.dim))'.*rand(cfg.dim,1);
        %
        cfg.start_coords = [-2.5;0;0;0;-1;0;0];  % [q1; q2]
        cfg.end_coords = [2.7;0;0;0;0;0;0];
    end

collided1 = checkPath_single(cfg.start_coords(1:cfg.dim), obs, cfg.y_f, cfg.circle_obs, cfg.dim);
collided2 = checkPath_single(cfg.end_coords(1:cfg.dim), obs, cfg.y_f, cfg.circle_obs, cfg.dim);
end

for RRT_star = 0
    cfg.RRT_star = RRT_star;
    for NN_check = 1
        cfg.NN_check = NN_check;
        fprintf('RRT_star = %d \nNN_check = %d \n\n', cfg.RRT_star,cfg.NN_check);
        
        if cfg.NN_check
            cfg.grad_heuristic = 0;
        else
            cfg.grad_heuristic = 0;
        end
        
        close all
        
        
        for i = 1:1
            % Plan a path
            path = RRT(@()(RandomSample(obs,cfg)), @Dist, ...
                obs, cfg);
        end
    end
end


