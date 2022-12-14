function path = RRT (RandomSample, Dist, obs, cfg)
% RRT - Rapidly-exploring Random Trees : This procedure computes a path
% using RRT. It relies on 2 functions:
% RandomSample which generates the coordinate vector for a random sample in free space.
% Dist which measures the distance between two coordinate vectors.
%
% Inputs :
%
%   RandomSample : A function that returns a random sample in freespace
%
%   Dist : A function that computes the distance between a given point in
%        configuration space and all of the entries in an array of samples
%
%   obstacle : Obstacle
%
%   cfg.vertex_coords : Start and end point.
%
%   cfg.dim : Two or three link robot.
%
%   cfg.display : Whether to visualize the planning process
%
%   cfg.stepsize : Predefined step-size distance used to create new node.
%
%   cfg.disTh : Acceptable end error
%
%   cfg.maxFailedAttempts : Maximum number of cycles
%
%   NN_check : RRT or RRT_NN_check
%
% Output :
%    path : An array containing the linear indices of the cells along the
%    shortest path from start to dest or an empty array if there is no
%    path.

pos_enc = @(x)[x sin(x) cos(x)];

% initialize the figure handle
% Visualization
if cfg.display1 || cfg.display2 || cfg.display3
    handle = visual_init(obs, cfg, cfg.y_f);
    handle.Point_1 = [];
    handle.Point_2 = [];
    handle.Point_22 = [];
    handle.Point_3 = [];
    handle.Quiver_1 = [];
    handle.Quiver_2 = [];
    handle.Quiver_3 = [];
    handle.r_h_dy = [];
    handle.anime = [];
else
    handle = [];
end

RRTree = double([cfg.start_coords; 0; -1]); % The index of the starting point is set to - 1,
% and the cost is 0. Pos: [q1;q2]
failedAttempts = 0;  % Number of failures
pathFound=false;  % Whether the path was found

counter_rand = 0;   % Number of sampling

Time = 0;
tic();
if cfg.saveRand
    rand_state = [];
    rng(1,'twister');
end

Joint = [];
while failedAttempts <= cfg.maxFailedAttempts && counter_rand <= cfg.maxSample
    %% obtain P_new
    if cfg.saveRand
        if exist('RRT/rand_state.mat','file')
            load('RRT/rand_state.mat')
        end
        if ~ exist('RRT/rand_state.mat','file')
            if rand <= 0.9   % Sampling randomly or moving towards the target
                rand_state = [rand_state; rng];
                sample = RandomSample();    %[q1;q2]
                rand_state = [rand_state; rng];
                counter_rand = counter_rand + 1;
            else
                rand_state = [rand_state; rng];
                sample = cfg.end_coords;
                rand_state = [rand_state; rng];
                counter_rand = counter_rand + 1;
            end
        else
            if rand <= 0.9   % Sampling randomly or moving towards the target
                rng(rand_state(2*counter_rand + 1));
                sample = RandomSample();    %[q1;q2]
                rng(rand_state(2*counter_rand + 2));
                counter_rand = counter_rand + 1;
            else
                sample = cfg.end_coords;
                rng(rand_state(2*counter_rand + 2));
                counter_rand = counter_rand + 1;
            end
        end
    else
        if rand <= 1   % Sampling randomly or moving towards the target
            sample = RandomSample();    %[q1;q2]
            counter_rand = counter_rand + 1;
        else
            sample = cfg.end_coords;
            counter_rand = counter_rand + 1;
        end
    end
    
    %% Find the closest node.
    % Calculate the distance from sample to each nodes in RRT tree.
    [closestNode, closestNode_int, M, I, T] = nearest_kd(@Dist_kd, sample, RRTree, cfg);
    
    %% Calculate the new node
    newPoint = Steer(closestNode, sample, T, cfg);
    % newPoint include [a_1, v_limit, a_2, t_1, t_v, t_2]
    
    %% collision checking
    % Chech wheter the path from closest node to new node is available
    [collided, collidedPose, ObsPoint] = checkPath_kd(closestNode, newPoint, obs, cfg.dim, cfg.NN_check, cfg.y_f, cfg.circle_obs, cfg);
    
    %% if cfg.grad_heuristic = Ture, obtain a new p_new based on grad heuristic when collisied,
    % otherwise, discard it directly %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if collided
        % Update newPoint is cfg.grad_heuristic = Ture
        if cfg.grad_heuristic && closestNode_int > 0
            [newPoint_RRTstar, Joint] = NewNodeGrad(pos_enc, closestNode, ...
                collidedPose, ObsPoint, closestNode_int, RRTree, cfg);
        else
            failedAttempts = failedAttempts+1;
            continue;
        end
        
        % visualize collied point
        if cfg.display1 || cfg.display2 || cfg.display3
            handle = display_when_collied(handle, cfg, collidedPose, ObsPoint, Joint, closestNode_int, newPoint_RRTstar);
        end
        
        % update newPoint
        if cfg.grad_heuristic && closestNode_int > 0
            if sum(newPoint_RRTstar' < cfg.q_min(1:cfg.dim)) || sum(newPoint_RRTstar' > cfg.q_max(1:cfg.dim))
                collided = 1;
            else
                [collided, ~, ~] = checkPath(closestNode, newPoint_RRTstar, obs, cfg.dim, cfg.NN_check, cfg.y_f, cfg.circle_obs);
            end
        end
        if collided
            failedAttempts = failedAttempts+1;
            continue;
        else
            newPoint = newPoint_RRTstar;
        end
        
    end
    
    %% Add p_new to RRT Tree
    %% Check whether the new node is near the target
    if Dist_kd(sample,cfg.end_coords,cfg) < cfg.disTh
        pathFound = true;
        break;
    end
    
    %% Check whether the new node is in the RRT tree
    %     [~, I2] = min(Dist_kd(sample, RRTree, cfg), [], 2);
    %     if Dist_kd(sample, RRTree(:, I2), cfg) < cfg.disTh
    %         failedAttempts = failedAttempts+1;
    %         continue;
    %     end
    
    %% RRT* or RRT
    if cfg.RRT_star
        [RRTree, failedAttempts] = run_RRT_star(RRTree, closestNode, sample, newPoint, I, T, obs, cfg, handle);
    else
        RRTree = IntermediateStates(RRTree, closestNode, sample, newPoint, I, T, obs, cfg, handle);
        failedAttempts = 0;
    end
end

% Add the path from final node to target
if cfg.display2 && cfg.dim == 2 && pathFound
    plot_curve_kd(closestNode, cfg.end_coords, cfg, handle);
end

if cfg.display3 == 1
    tmp_new = calc_fk(closestNode(1:cfg.dim),cfg.r,cfg.d,cfg.alpha,cfg.base); % change [q2 q1] to [q1 q2]
    tmp_close = calc_fk(cfg.end_coords(1:cfg.dim),cfg.r,cfg.d,cfg.alpha,cfg.base);
    line(handle.ax_h3,[tmp_close(end,1);tmp_new(end,1)],[tmp_close(end,2);tmp_new(end,2)]);
end

% counter = counter + 1;

if pathFound
    disp('Found the path!');
elseif ~pathFound
    disp('No path found. maximum attempts reached');
    path = [];
end

if cfg.saveRand
    save RRT/rand_state rand_state
end

%% Visualize the found path
if pathFound
    path = [cfg.end_coords];
    prev = I;
    while prev > 0
        path = [RRTree(1:end - 2, prev), path];
        if ~ismember(prev, [])
            if cfg.display2 || cfg.display3
                plot_curve_kd(path(1:2*cfg.dim,1), path(1:2*cfg.dim,2), cfg, handle, [1 0 0], 2);
            end
            
            
            prev = RRTree(end, prev);
        end
    end
    
    if cfg.display5
        if cfg.dim == 2
%             MaxTime = RRTree(end-1, I);
            
            % create figure q_1
            fig_handle5 = figure('Name','q_1','Position',[200 500 400 400]);
            ax_h5 = axes(fig_handle5,'View',[0 90]);
            axis equal
            hold on
%             ax_h5.XLim = [0 1.3*MaxTime];
            ax_h5.YLim = [cfg.q_min(1) cfg.q_max(1)];
            
            % create figure q_2
            fig_handle6 = figure('Name','q_2','Position',[200 10 400 400]);
            ax_h6 = axes(fig_handle6,'View',[0 90]);
            axis equal
            hold on
%             ax_h6.XLim = [0 1.3*MaxTime];
            ax_h6.YLim = [cfg.q_min(1) cfg.q_max(1)];
            
            % plot DOF-DOF-dot
            t_end1 = 0;
            t_end2 = 0;
            for i = 1 : size(path,2)-1
                t_end1 = plot_qdot(path(:,i), path(:,i+1), cfg, 1, t_end1, ax_h5);
                t_end2 = plot_qdot(path(:,i), path(:,i+1), cfg, 2, t_end2, ax_h6);
            end
        else
%             MaxTime = RRTree(end-1, I);
            
            % create figure q_1
            fig_handle5 = figure('Name','q_1','Position',[200 500 400 400]);
            ax_h5 = axes(fig_handle5,'View',[0 90]);
            axis equal
            hold on
%             ax_h5.XLim = [0 1.3*MaxTime];
            ax_h5.YLim = [cfg.q_min(1) cfg.q_max(1)];
            
            % create figure q_2
            fig_handle6 = figure('Name','q_2','Position',[200 10 400 400]);
            ax_h6 = axes(fig_handle6,'View',[0 90]);
            axis equal
            hold on
%             ax_h6.XLim = [0 1.3*MaxTime];
            ax_h6.YLim = [cfg.q_min(2) cfg.q_max(2)];
            
            % create figure q_3
            fig_handle7 = figure('Name','q_3','Position',[200 10 400 400]);
            ax_h7 = axes(fig_handle7,'View',[0 90]);
            axis equal
            hold on
%             ax_h7.XLim = [0 1.3*MaxTime];
            ax_h7.YLim = [cfg.q_min(3) cfg.q_max(3)];
            
            % create figure q_4
            fig_handle8 = figure('Name','q_4','Position',[200 10 400 400]);
            ax_h8 = axes(fig_handle8,'View',[0 90]);
            axis equal
            hold on
%             ax_h8.XLim = [0 1.3*MaxTime];
            ax_h8.YLim = [cfg.q_min(4) cfg.q_max(4)];
            
            % create figure q_5
            fig_handle9 = figure('Name','q_5','Position',[200 10 400 400]);
            ax_h9 = axes(fig_handle9,'View',[0 90]);
            axis equal
            hold on
%             ax_h9.XLim = [0 1.3*MaxTime];
            ax_h9.YLim = [cfg.q_min(5) cfg.q_max(5)];
            
            % create figure q_6
            fig_handle10 = figure('Name','q_6','Position',[200 10 400 400]);
            ax_h10 = axes(fig_handle10,'View',[0 90]);
            axis equal
            hold on
%             ax_h10.XLim = [0 1.3*MaxTime];
            ax_h10.YLim = [cfg.q_min(6) cfg.q_max(6)];
            
            % create figure q_7
            fig_handle11 = figure('Name','q_7','Position',[200 10 400 400]);
            ax_h11 = axes(fig_handle11,'View',[0 90]);
            axis equal
            hold on
%             ax_h11.XLim = [0 1.3*MaxTime];
            ax_h11.YLim = [cfg.q_min(7) cfg.q_max(7)];
            
            % plot DOF-DOF-dot
            t_end1 = 0;
            t_end2 = 0;
            t_end3 = 0;
            t_end4 = 0;
            t_end5 = 0;
            t_end6 = 0;
            t_end7 = 0;
            for i = 1 : size(path,2)-1
                t_end1 = plot_qdot(path(:,i), path(:,i+1), cfg, 1, t_end1, ax_h5);
                t_end2 = plot_qdot(path(:,i), path(:,i+1), cfg, 2, t_end2, ax_h6);
                t_end3 = plot_qdot(path(:,i), path(:,i+1), cfg, 3, t_end3, ax_h7);
                t_end4 = plot_qdot(path(:,i), path(:,i+1), cfg, 4, t_end4, ax_h8);
                t_end5 = plot_qdot(path(:,i), path(:,i+1), cfg, 5, t_end5, ax_h9);
                t_end6 = plot_qdot(path(:,i), path(:,i+1), cfg, 6, t_end6, ax_h10);
                t_end7 = plot_qdot(path(:,i), path(:,i+1), cfg, 7, t_end7, ax_h11);
            end
        end
    end
    Time = Time + toc();
    
    
    % moving the robot
    if cfg.display3 && cfg.anime
        q_list = [];
        for i = 1:size(path,2)-1
            q_stage = plot_anime(path(:,i), path(:,i+1), cfg);
            q_list = [q_list, q_stage];
        end
        for i = 1:size(q_list,2)
            if ~isempty(handle.anime)
                set(handle.anime,'visible','off');
            end
            figure(handle.fig_handle3);
            hold on
            handle.anime = create_r(handle.ax_h3,q_list(:,i), cfg.r, cfg.d, cfg.alpha, cfg.base);
            pause(0.1);
            hold off
        end
    end
    
    % Calculate the length of planned path
    pathLength = 0;
    for i = 1:size(path,2)-1
        pathLength = pathLength + Dist_kd(path(:, i+1),path(:, i),cfg);
    end
    
    fprintf('Runing time = %d \nNumber of iteration = %d \nNumber of expended nodes = %d \nPath Length = %d \n\n', Time, counter_rand,size(RRTree,2),pathLength);   % ??????????????????toc???????????????
    
    % save data
    if exist('results/result.mat')
        load('results/result.mat');
        result(end+1,:) = [cfg.start_coords', cfg.end_coords', Time, counter_rand, size(RRTree,2), pathLength];
    else
        result = [cfg.start_coords', cfg.end_coords', Time, counter_rand, size(RRTree,2), pathLength];
    end
    save 'results/result.mat' result
    
    if ~cfg.display1 && ~cfg.display2 && ~cfg.display3 && cfg.display4
        if cfg.dim == 2
            cfg.display2 = 1;
        end
        cfg.display3 = 1;
        handle = visual_init(obs, cfg, cfg.y_f);
        handle.anime = [];
        
        % plot the progress of planning
        for i = 2 : size(RRTree,2)
            plot_curve_kd(RRTree(1:2*cfg.dim,RRTree(end,i)), RRTree(1:2*cfg.dim,i), cfg, handle);
        end
        
        % add final path
        if cfg.display2 && cfg.dim == 2
            plot_curve_kd(closestNode, cfg.end_coords, cfg, handle);
        end
        if cfg.display3 == 1
            tmp_new = calc_fk(closestNode(1:cfg.dim),cfg.r,cfg.d,cfg.alpha,cfg.base); % change [q2 q1] to [q1 q2]
            tmp_close = calc_fk(cfg.end_coords(1:cfg.dim),cfg.r,cfg.d,cfg.alpha,cfg.base);
            line(handle.ax_h3,[tmp_close(end,1);tmp_new(end,1)],[tmp_close(end,2);tmp_new(end,2)]);
        end
        
        % plot planned path
        for i = 1 : size(path,2)-1
            if cfg.display2 || cfg.display3
                plot_curve_kd(path(1:2*cfg.dim,i), path(1:2*cfg.dim,i+1), cfg, handle, [1 0 0], 2);
            end
        end
        
        % moving the robot
        if cfg.display3 && cfg.anime
            q_list = [];
            for i = 1:size(path,2)-1
                q_stage = plot_anime(path(:,i), path(:,i+1), cfg);
                q_list = [q_list, q_stage];
            end
            for i = 1:size(q_list,2)
                if ~isempty(handle.anime)
                    set(handle.anime,'visible','off');
                end
                figure(handle.fig_handle3);
                hold on
                handle.anime = create_r(handle.ax_h3,q_list(:,i), cfg.r, cfg.d, cfg.alpha, cfg.base);
                pause(0.1);
                hold off
            end
        end
    end
else
    Time = Time + toc();
    fprintf('Runing time = %d \nNumber of iteration = %d \nNumber of expended nodes = %d \nPath Length = %d \n\n', Time, counter_rand,size(RRTree,2),0);   % ??????????????????toc???????????????
    
    %     save data
    if exist('results/result.mat')
        load('results/result.mat');
        result(end+1,:) = [cfg.start_coords', cfg.end_coords', Time, 10000, size(RRTree,2), 10000];
    else
        result = [cfg.start_coords', cfg.end_coords', Time, 10000, size(RRTree,2), 10000];
    end
    save 'results/result.mat' result
end
end
%%

function pts = calc_fk(j_state,r,d,alpha,base)
P = dh_fk(j_state,r,d,alpha,base);
pts = zeros(3,3);
for i = 1:1:length(j_state) + 1
    v = [0,0,0];
    R = P{i}(1:3,1:3);
    T = P{i}(1:3,4);
    p = v*R'+T';
    pts(i,:) = p;
end
end

function handle = create_r(ax_h,j_state,r,d,alpha,base)
pts = calc_fk(j_state,r,d,alpha,base);
hold on
handle = plot(ax_h,pts(:,1),pts(:,2),'LineWidth',2,...
    'Marker','o','MarkerFaceColor','k','MarkerSize',4, 'Color',[0, 0, 1]);
hold off
end

function move_r(r_handle,j_state,r,d,alpha,base)
pts = calc_fk(j_state,r,d,alpha,base);
r_handle.XData = pts(:,1);
r_handle.YData = pts(:,2);
end

function handle = display_when_collied(handle, cfg, collidedPose, ObsPoint, Joint, closestNode_int, newPoint_RRTstar)
pos_enc = @(x)[x sin(x) cos(x)];
% Visualize collision
if cfg.display1
    % moving the robot
    move_r(handle.r_h,collidedPose,cfg.r,cfg.d,cfg.alpha,cfg.base)
    
    x_span = linspace(cfg.q_min(end - 1),cfg.q_max(end - 1));
    y_span = linspace(cfg.q_min(end),cfg.q_max(end));
    [X_mg,Y_mg] = meshgrid(x_span, y_span);
    x = [X_mg(:) Y_mg(:)]';
    inp_1 = pos_enc([repmat(collidedPose',[length(x),1])'; x]')';
    inp_11 = pos_enc([collidedPose', ObsPoint'])';
    val_1_y = cfg.y_f(inp_1);
    val_1_dy = cfg.dy_f(inp_11);
    if ~isempty(handle.Point_1)
        set(handle.Point_1,'visible','off');
        set(handle.Quiver_1,'visible','off');
    end
    
    % Visualize direction computed using NN
    direction = 100*(val_1_dy(cfg.dim + 1 : cfg.dim + 2)+...
        val_1_dy(2*cfg.dim + 3 : 2*cfg.dim + 4) .* cos([ObsPoint(1),ObsPoint(2)])+...
        val_1_dy(3*cfg.dim + 5 : 3*cfg.dim + 6) .* (-sin([ObsPoint(1),ObsPoint(2)])))/3;
    if cfg.grad_heuristic
        figure(handle.fig_handle);
        hold on
        handle.Point_1 = plot(ObsPoint(1),ObsPoint(2),'*r');
        handle.Quiver_1 = quiver(ObsPoint(1),ObsPoint(2),direction(1),direction(2));
        hold off
    end
    
    Z_mg = reshape(val_1_y,size(X_mg));
    handle.cr1.ZData = Z_mg;
    handle.ctr.ZData = Z_mg;
    handle.ctr.LevelList=linspace(-3,15);
end

% plot collided pos in configuration space
if cfg.display2 && cfg.dim == 2
    figure(handle.fig_handle2)
    hold on
    if ~isempty(handle.Point_2)
        set(handle.Point_2,'visible','off');
        set(handle.Quiver_2,'visible','off');
    end
    handle.Point_2 = plot(handle.ax_h2, collidedPose(2),collidedPose(1),'*r');
    if cfg.NN_check
        if cfg.grad_heuristic
            handle.Quiver_2 = quiver(collidedPose(2),collidedPose(1),Joint(2),Joint(1));
        end
    end
    hold off
    
    %%%%%%%%%%%%%%%%%%%%%%%% Update newPoint is cfg.grad_heuristic = Ture %%%%%%%%%%%%%%%%%%%%%%%%
    if cfg.grad_heuristic && closestNode_int > 0
        hold on
        if ~isempty(handle.Point_22)
            set(handle.Point_22,'visible','off');
        end
        %                 quiver(closestNode(1),closestNode(2),d_vector(1),d_vector(2));
        %                 quiver(closestNode(1),closestNode(2),projection(1),projection(2));
        handle.Point_22 = plot(handle.ax_h2,newPoint_RRTstar(2),newPoint_RRTstar(1),'or');
        hold off
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end

% plot collided point in task space
if cfg.display3
    tmp_3 = calc_fk(collidedPose,cfg.r,cfg.d,cfg.alpha,cfg.base);
    figure(handle.fig_handle3);
    hold on
    if ~isempty(handle.Point_3)
        set(handle.Point_3,'visible','off');
    end
    handle.Point_3 = plot(handle.ax_h3,tmp_3(end,1),tmp_3(end,2),'*r');
    hold off
end
end

function [newPoint_RRTstar, Joint] = NewNodeGrad(pos_enc, closestNode, collidedPose, ObsPoint, closestNode_int, RRTree, cfg)

inp = pos_enc([collidedPose', ObsPoint'])';
dy_val = cfg.dy_f(inp);
Joint = (dy_val(1 : cfg.dim)+...
    dy_val(cfg.dim + 3 : 2*cfg.dim + 2) .* cos(collidedPose')+...
    dy_val(2*cfg.dim + 5 : 3*cfg.dim + 4) .* (-sin(collidedPose')))/3;

q_prev = RRTree(1:cfg.dim, closestNode_int);    % [q1,q2]
d_vector = closestNode - q_prev;    % [q1,q2]
dGamma = Joint;   % [q1,q2]
%                                             projection = -(dot(dGamma', d_vector))/(dot(d_vector, d_vector))*d_vector;
projection = (dot(d_vector, dGamma'))/(dot(dGamma', dGamma'))*dGamma';  %[q2,q1]
newPoint_RRTstar = closestNode + cfg.stepsize * (d_vector - projection)/norm(d_vector - projection);    %[q2,q1]

end

%%
function [RRTree, failedAttempts] = run_RRT_star(RRTree, closestNode, sample, newPoint, I, T, obs, cfg, handle)
%         search nodes in a circle of radius 'cfg.RadiusForNeib'
[Idx,D] = rangesearch(RRTree(1:cfg.dim,:)',newPoint',cfg.RadiusForNeib);
[Idx,D] = rangesearch_kd(RRTree(1:cfg.dim,:)',newPoint',cfg.RadiusForNeib);
nearIndexList = Idx{1};
disToNewList = D{1};

temp_cost = RRTree(cfg.dim+1,I) + cfg.stepsize;
temp_parent = I;
x_mincost = closestNode;
% select the parent of x_new to minmize the cost
for cost_index = 1:length(nearIndexList)    % cost_index?????????disToNewList?????????,????????????????????????
    costToNew = disToNewList(cost_index) + RRTree(cfg.dim+1,nearIndexList(cost_index));
    if costToNew * 1.00001 < temp_cost    % temp_cost?????????minDist??????????????????cost
        x_mincost = RRTree(1:cfg.dim, nearIndexList(cost_index));   % ?????????????????????????????????
        [collided, ~, ~] = checkPath(x_mincost, newPoint, obs, cfg.dim, cfg.NN_check, cfg.y_f, cfg.circle_obs);
        if collided
            continue;   %????????????
        end
        temp_cost = costToNew;
        temp_parent = nearIndexList(cost_index);
    end
end

RRTree = [RRTree, [newPoint; temp_cost; temp_parent]];
failedAttempts = 0;

% Visualize the expanding process of RRT
% configuration sapce
if cfg.display2 && cfg.dim == 2
    line(handle.ax_h2, [RRTree(2,temp_parent);newPoint(2)],[RRTree(1,temp_parent);newPoint(1)],'color',[0 0 1]);
    drawnow
end

%task space
if cfg.display3
    tmp_new = calc_fk(newPoint,cfg.r,cfg.d,cfg.alpha,cfg.base); % change [q2 q1] to [q1 q2]
    tmp_close = calc_fk(x_mincost,cfg.r,cfg.d,cfg.alpha,cfg.base);
    line(handle.ax_h3, [tmp_close(end,1);tmp_new(end,1)],[tmp_close(end,2);tmp_new(end,2)],'color',[0 0 1]);
    drawnow
end


% rewire
for rewire_index = 1:length(nearIndexList)
    if nearIndexList(rewire_index) ~= temp_parent    % ??????????????????????????????cost?????????
        newCost = temp_cost + disToNewList(rewire_index);    % ??????neib??????x_new?????????????????????
        if newCost < RRTree(cfg.dim+1, nearIndexList(rewire_index))    % ????????????
            x_neib = RRTree(1:cfg.dim, nearIndexList(rewire_index));
            [collided, ~, ~] = checkPath(x_neib, newPoint, obs, cfg.dim, cfg.NN_check, cfg.y_f, cfg.circle_obs);
            if collided
                continue;   %????????????
            end
            RRTree(cfg.dim + 1, nearIndexList(rewire_index)) = newCost;
            
            Parent_int = RRTree(end,nearIndexList(rewire_index));
            if cfg.display2 && cfg.dim == 2
                line(handle.ax_h2,[x_neib(2);newPoint(2)],[x_neib(1);newPoint(1)],'color',[0 1 0]);
                line(handle.ax_h2,[x_neib(2);RRTree(2,Parent_int)],[x_neib(1);RRTree(1,Parent_int)],'color',[1 1 1]);
                drawnow
            end
            
            RRTree(end, nearIndexList(rewire_index)) = size(RRTree,2);
            
        end
    end
end
end