clc
close all force
clear all

addpath('RRT');
addpath('RRT/tradtion_check');
addpath('data');
addpath('data_7d');

% parameters for control
cfg.dim = 2;    % planning in cfg.dim-D case.
cfg.circle_obs = false;  % when equals to true, assuming the obstacle as a 
                      % sphere;  when equals to false, dividing it to many points.
cfg.display1 = false;
cfg.display2 = false;
cfg.display3 = false;   % whether to plot figure 1, 2, 3 during planning.
cfg.display4 = false;    % whether to plot figure after planning.
cfg.save_fig = false;
cfg.maxSample = 2000;  % the maximum number of random samples.

cfg.stepsize = 0.1 * pi;    % 0.095 * pi;
cfg.disTh = 0.1 * pi;
cfg.maxFailedAttempts = 2000;
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
    k1 = 0.95; k2 = 0.95;
    cfg.q_min = [-k1*pi, -k2*pi, -10, -10];
    cfg.q_max = [k1*pi, k2*pi, 10, 10];
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

for Obs_int = 1
    % Add obstacles
    % load('obs_data.mat');
    [obs_data] = xlsread('obs_data.xlsx');
    if cfg.circle_obs
        obs.centers = obs_data(:,1:2);
        obs.radiuses = obs_data(:,3);

        %     obs.vertices = [4 3; 4 4; 5 3; 5 4 ; -5 5; -5 7; -7 5; -7 7; -8 -6; 8 -6; -8 -10; 8 -10];
        %     obs.faces = [1 2 3; 2 3 4 ; 5 6 7; 6 7 8; 9 10 11; 10 11 12];
        % else
        %     obs.vertices = [4 3; 4 4; 5 3; 5 4 ; -5 5; -5 7; -7 5; -7 7; -8 -6; 8 -6; -8 -10; 8 -10];
        %     obs.faces = [1 2 3; 2 3 4 ; 5 6 7; 6 7 8; 9 10 11; 10 11 12];

%         obs = generate_obs(5, 10, 1.5, 0.5, cfg);
    end

    for State_int = 1
        % Add start and ending pose
        collided1 = 1;
        collided2 = 1;
        while collided1 || collided2
            if cfg.dim == 2
%                 cfg.start_coords = cfg.q_min(1:cfg.dim)' + (cfg.q_max(1:cfg.dim)-cfg.q_min(1:cfg.dim))'.*rand(cfg.dim,1);
%                 cfg.end_coords = cfg.q_min(1:cfg.dim)' + (cfg.q_max(1:cfg.dim)-cfg.q_min(1:cfg.dim))'.*rand(cfg.dim,1);

                cfg.start_coords = [-2.5; -1];  % [q1; q2]
                cfg.end_coords = [2.7; 0];
            elseif cfg.dim == 7
        %         cfg.start_coords = cfg.q_min(1:cfg.dim)' + (cfg.q_max(1:cfg.dim)-cfg.q_min(1:cfg.dim))'.*rand(cfg.dim,1);
        %         cfg.end_coords = cfg.q_min(1:cfg.dim)' + (cfg.q_max(1:cfg.dim)-cfg.q_min(1:cfg.dim))'.*rand(cfg.dim,1);
        %         
                cfg.start_coords = [-2.5;0;0;0;-1;0;0];  % [q1; q2]
                cfg.end_coords = [2.7;0;0;0;0;0;0];
            end

            collided1 = checkPath_single(cfg.start_coords, obs, cfg.y_f, cfg.circle_obs, cfg.dim);
            collided2 = checkPath_single(cfg.end_coords, obs, cfg.y_f, cfg.circle_obs, cfg.dim);
        end

        for RRT_star = 0
            cfg.RRT_star = RRT_star;
            for grad_heuristic = 1
                cfg.grad_heuristic = grad_heuristic;
                cfg.NN_check = 1;
                fprintf('RRT_star = %d \nHeuristic = %d \n\n', cfg.RRT_star,cfg.grad_heuristic);
                
                close all
                clear result


                for i = 1
                    Name_result = ['results/Obs', num2str(Obs_int), '_State', num2str(State_int), ...
                        '_RRTStar', num2str(cfg.RRT_star), '_Heuristic', num2str(cfg.grad_heuristic), '.mat'];
                    % Add data
                    if exist(Name_result)
                        load(Name_result);
                        result(end + 1, :).obs = obs;
                        result(end, :).start_coords = cfg.start_coords;
                        result(end, :).end_coords = cfg.end_coords;
                    else
                        result(1).obs = obs;
                        result(1).start_coords = cfg.start_coords;
                        result(1).end_coords = cfg.end_coords;
                    end
                    save(Name_result, 'result');
                    
                    % Plan a path
                    path = RRT(@()(RandomSample(obs,cfg.q_min,cfg.q_max,cfg.dim,cfg)), @Dist, ...
                        obs, cfg, Name_result);
                end
            end
        end
    end
end

