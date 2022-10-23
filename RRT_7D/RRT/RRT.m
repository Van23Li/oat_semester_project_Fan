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
            if rand <= 0.5   % Sampling randomly or moving towards the target
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
            if rand <= 0.5   % Sampling randomly or moving towards the target
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
        if rand <= 0.5   % Sampling randomly or moving towards the target
            sample = RandomSample();    %[q1;q2]
            counter_rand = counter_rand + 1;
        else
            sample = cfg.end_coords;
            counter_rand = counter_rand + 1;
        end
    end
    %% Find the closest node.
    % Calculate the distance from sample to each nodes in RRT tree.
    [closestNode, closestNode_int, M, I] = nearest(@Dist,sample, RRTree, cfg);
    
    %% Calculate the new node
    if M > cfg.stepsize
        newPoint = closestNode + (sample-closestNode)/norm(sample-closestNode, 2) * cfg.stepsize;
    else
        newPoint = sample;  % [q1; q2]
    end
    
    %% collision checking
    % Chech wheter the path from closest node to new node is available
    [collided, collidedPose, ObsPoint] = checkPath(closestNode, newPoint, obs, cfg.dim, cfg.NN_check, cfg.y_f, cfg.circle_obs);
    
    %% if cfg.grad_heuristic = Ture, obtain a new p_new based on grad heuristic when collisied,
    % otherwise, discard it directly %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if collided
        % Update newPoint is cfg.grad_heuristic = Ture
        if cfg.grad_heuristic && closestNode_int > 0
            [newPoint_RRTstar, Joint] = NewNodeGrad(pos_enc, closestNode, ...
                collidedPose, ObsPoint, closestNode_int, RRTree, cfg);
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
    if Dist(newPoint,cfg.end_coords) < cfg.disTh
        pathFound = true;
        break;
    end
    
    %% Check whether the new node is in the RRT tree
    [~, I2] = min(Dist(newPoint, RRTree(1:cfg.dim, :)), [], 2);
    if Dist(newPoint, RRTree(1:cfg.dim, I2)) < cfg.disTh
        failedAttempts = failedAttempts+1;
        continue;
    end
    
    %% RRT* or RRT
    if cfg.RRT_star
        [RRTree, failedAttempts] = run_RRT_star(RRTree, newPoint, I, closestNode, obs, cfg);
    else
        % Add the new node to the RRT tree
        RRTree = [RRTree, [newPoint; RRTree(cfg.dim+1,I) + cfg.stepsize ;I]];
        failedAttempts = 0;
        
        % Visualize the expanding process of RRT
        % plot in configuration space
        if cfg.display2 && cfg.dim == 2
            line(handle.ax_h2,[closestNode(2);newPoint(2)],[closestNode(1);newPoint(1)],'color',[0 0 1]);
            drawnow
        end
        % plot in task space
        if cfg.display3
            tmp_new = calc_fk(newPoint,cfg.r,cfg.d,cfg.alpha,cfg.base);
            tmp_close = calc_fk(closestNode,cfg.r,cfg.d,cfg.alpha,cfg.base);
            line(handle.ax_h3,[tmp_close(end,1);tmp_new(end,1)],[tmp_close(end,2);tmp_new(end,2)],'color',[0 0 1]);
            drawnow
        end
        
    end
end

% Add the path from final node to target
if cfg.display2 && cfg.dim == 2 && pathFound
    line(handle.ax_h2,[closestNode(2);cfg.end_coords(2)],[closestNode(1);cfg.end_coords(1)]);
end
if cfg.display3 == 1
    tmp_new = calc_fk(closestNode,cfg.r,cfg.d,cfg.alpha,cfg.base); % change [q2 q1] to [q1 q2]
    tmp_close = calc_fk(cfg.end_coords,cfg.r,cfg.d,cfg.alpha,cfg.base);
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
        path = [RRTree(1:cfg.dim, prev), path];
        if ~ismember(prev, [])
            if cfg.display2 == 1 && cfg.dim == 2
                line(handle.ax_h2,path(2,1:2),path(1,1:2),'color',[1 0 0],'linewidth',2);
            end
            if cfg.display3 == 1
                tmp_new = calc_fk(path(:,1),cfg.r,cfg.d,cfg.alpha,cfg.base); % change [q2 q1] to [q1 q2]
                tmp_close = calc_fk(path(:,2),cfg.r,cfg.d,cfg.alpha,cfg.base);
                line(handle.ax_h3,[tmp_close(end,1);tmp_new(end,1)],[tmp_close(end,2);tmp_new(end,2)],'color',[1 0 0],'linewidth',2);
            end
            prev = RRTree(end, prev);
        end
    end
    % anime
    if cfg.display3 == 1
        for i = 1:size(path,2)
            % moving the robot
            if ~isempty(handle.anime)
                set(handle.anime,'visible','off');
            end
            hold on
            handle.anime = create_r(handle.ax_h3,path(:,i), cfg.r, cfg.d, cfg.alpha, cfg.base);
            pause(0.2);
            hold off
        end
    end
    
    % Calculate the length of planned path
    pathLength = 0;
    for i = 1:size(path,2)-1
        pathLength = pathLength + Dist(path(1:cfg.dim, i),path(1:cfg.dim, i+1));
        
    end
    
    Time = Time + toc();
    fprintf('Runing time = %d \nNumber of iteration = %d \nNumber of expended nodes = %d \nPath Length = %d \n\n', Time, counter_rand,size(RRTree,2),pathLength);   % 打印运行时间toc和路径长度
    
    % save data
    %     if exist('results/result.mat')
    %         load('results/result.mat');
    %         result(end+1,:) = [cfg.start_coords', cfg.end_coords', Time, counter_rand, size(RRTree,2), pathLength];
    %     else
    %         result = [cfg.start_coords', cfg.end_coords', Time, counter_rand, size(RRTree,2), pathLength];
    %     end
    %     save 'results/result.mat' result
else
    Time = Time + toc();
    fprintf('Runing time = %d \nNumber of iteration = %d \nNumber of expended nodes = %d \nPath Length = %d \n\n', Time, counter_rand,size(RRTree,2),0);   % 打印运行时间toc和路径长度
    
    % save data
    %     if exist('results/result.mat')
    %         load('results/result.mat');
    %         result(end+1,:) = [cfg.start_coords', cfg.end_coords', Time, 10000, size(RRTree,2), 10000];
    %     else
    %         result = [cfg.start_coords', cfg.end_coords', Time, 10000, size(RRTree,2), 10000];
    %     end
    %     save 'results/result.mat' result
end

if ~cfg.display1 && ~cfg.display2 && ~cfg.display3 && cfg.display4
    cfg.display2 = 1;
    cfg.display3 = 1;
    handle = visual_init(obs, cfg, cfg.y_f);
    for i = 2 : size(RRTree,2)
        if cfg.dim == 2
            line(handle.ax_h2,[RRTree(2,i), RRTree(2,RRTree(4,i))],...
                [RRTree(1,i), RRTree(1,RRTree(4,i))],...
                'color',[0 0 1]);
        end
        tmp_new = calc_fk(RRTree(1:cfg.dim,i),cfg.r,cfg.d,cfg.alpha,cfg.base);
        tmp_close = calc_fk(RRTree(1:cfg.dim,RRTree(end,i)),cfg.r,cfg.d,cfg.alpha,cfg.base);
        
        line(handle.ax_h3,[tmp_close(end,1);tmp_new(end,1)],[tmp_close(end,2);tmp_new(end,2)],'color',[0 0 1]);
        drawnow
    end
    if cfg.dim == 2
        line(handle.ax_h2,[closestNode(2);cfg.end_coords(2)],[closestNode(1);cfg.end_coords(1)]);
    end
    
    for i = 1 : size(path,2)-1
        if cfg.dim == 2
            line(handle.ax_h2,path(2,i:i+1),path(1,i:i+1),'color',[1 0 0],'linewidth',2);
        end
        
        tmp_new = calc_fk(path(:,i),cfg.r,cfg.d,cfg.alpha,cfg.base);
        tmp_close = calc_fk(path(:,i+1),cfg.r,cfg.d,cfg.alpha,cfg.base);
        
        line(handle.ax_h3,[tmp_close(end,1);tmp_new(end,1)],[tmp_close(end,2);tmp_new(end,2)],'color',[1 0 0],'linewidth',2);
        drawnow
    end
    
    % anime
    handle.anime = [];
    for i = 1:size(path,2)
        % moving the robot
        if ~isempty(handle.anime)
            set(handle.anime,'visible','off');
        end
        hold on
        handle.anime = create_r(handle.ax_h3,path(:,i), cfg.r, cfg.d, cfg.alpha, cfg.base);
        pause(0.2);
        hold off
    end
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
handle = plot(ax_h,pts(:,1),pts(:,2),'LineWidth',2,...
    'Marker','o','MarkerFaceColor','k','MarkerSize',4);
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
function [RRTree, failedAttempts] = run_RRT_star(RRTree, newPoint, I, closestNode, obs, cfg)

%         search nodes in a circle of radius 'cfg.RadiusForNeib'
[Idx,D] = rangesearch(RRTree(1:cfg.dim,:)',newPoint',cfg.RadiusForNeib);
nearIndexList = Idx{1};
disToNewList = D{1};

temp_cost = RRTree(cfg.dim+1,I) + cfg.stepsize;
temp_parent = I;
x_mincost = closestNode;
% select the parent of x_new to minmize the cost
for cost_index = 1:length(nearIndexList)    % cost_index是基于disToNewList的索引,不是整棵树的索引
    costToNew = disToNewList(cost_index) + RRTree(cfg.dim+1,nearIndexList(cost_index));
    if costToNew * 1.00001 < temp_cost    % temp_cost为通过minDist节点的路径的cost
        x_mincost = RRTree(1:cfg.dim, nearIndexList(cost_index));   % 符合剪枝条件节点的坐标
        [collided, ~, ~] = checkPath(x_mincost, newPoint, obs, cfg.dim, cfg.NN_check, cfg.y_f, cfg.circle_obs);
        if collided
            continue;   %有障碍物
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
    if nearIndexList(rewire_index) ~= temp_parent    % 若不是之前计算的最小cost的节点
        newCost = temp_cost + disToNewList(rewire_index);    % 计算neib经过x_new再到起点的代价
        if newCost < RRTree(cfg.dim+1, nearIndexList(rewire_index))    % 需要剪枝
            x_neib = RRTree(1:cfg.dim, nearIndexList(rewire_index));
            [collided, ~, ~] = checkPath(x_neib, newPoint, obs, cfg.dim, cfg.NN_check, cfg.y_f, cfg.circle_obs);
            if collided
                continue;   %有障碍物
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