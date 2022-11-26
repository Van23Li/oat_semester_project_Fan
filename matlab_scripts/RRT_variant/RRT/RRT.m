function path = RRT (RandomSample, Dist, obstacle, param)
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
%   vertex_coords : Start and end point.
%
%   dim : Two or three link robot.
%
%   display : Whether to visualize the planning process
%
%   stepsize : Predefined step-size distance used to create new node.
%
%   disTh : Acceptable end error
%
%   maxFailedAttempts : Maximum number of cycles
%
%   NN_check : RRT or RRT_NN_check
%
% Output :
%    path : An array containing the linear indices of the cells along the
%    shortest path from start to dest or an empty array if there is no
%    path.

vis1 = 1;
vis2 = 1;
vis3 = 1;
vis4 = 1;

RRT_star = param.RRT_star;   %if True, using RRT*
NN_check = param.NN_check;   % if True, check collision using NN
grad_heuristic = param.grad_heuristic;   % if True, introduce grad heuristic. Note when grad_heuristic = 1,
% NN_check has to be setten to 1.
RadiusForNeib = 0.3;    % set the parament (search radius) of RRT*

% set parameters
vertex_coords = param.vertex_coords;
dim = param.dim;
display = param.display;
stepsize = param.stepsize;
disTh = param.disTh;
maxFailedAttempts = param.maxFailedAttempts;
q_min = param.q_min;
q_max = param.q_max;
q_init = param.q_init;

% initialize the figure handle
Point_2 = [];
Point_3 = [];
Point_4 = [];
Quiver_2 = [];
Quiver_3 = [];
Quiver_4 = [];
r_h_dy = [];

%gmm model creation
load('data/net50_pos_thr.mat');
[y_f, dy_f] = tanhNN(net);

%parameters definition
global joint_state
r = [4 4 0];
d = [0 0 0];
alpha = [0 0 0];
base = eye(4);
joint_state = q_init;

%figure with robot
fig_handle = figure('Name','visualize grade','Position',[1000 10 450 353]);
ax_h = axes(fig_handle,'View',[0 90]);
axis equal
hold on
ax_h.XLim = [-2.2 2.2];
ax_h.YLim = [-2.2 2.2];
ax_h.ZLim = [-0.1 0.1];

%create contour object
x_span = linspace(q_min(3),q_max(3));
y_span = linspace(q_min(4),q_max(4));
[X_mg,Y_mg] = meshgrid(x_span, y_span);
[~, ctr] = contourf(ax_h, X_mg,Y_mg,0*X_mg,100,'LineStyle','none');
lvl = 0.5;
[~, cr1] = contour(ax_h, X_mg,Y_mg,X_mg-10,[lvl,lvl+0.001],'LineStyle','-','LineColor','k','LineWidth',2);

%create robot
r_h = create_r(ax_h,joint_state(1:2),r,d,alpha,base);

axes_rob = get(fig_handle,'CurrentAxes');
axes_rob.XLim = [q_min(3), q_max(3)];
axes_rob.YLim = [q_min(4), q_max(4)];

hold on;
obs = patch(obstacle,'FaceAlpha',0.4,'EdgeColor','none','LineStyle','none');
% load('RRT/ColliedPoints.mat');
% scatter(ColliedPoints(:,1),ColliedPoints(:,2),'filled');
hold off;


start_coords = vertex_coords(:, 1);
end_coords = vertex_coords(:, 2);

RRTree = double([start_coords; 0; -1]); % The index of the starting point is set to - 1,
% and the cost is 0.
failedAttempts = 0;  % Number of failures
% counter = 1;  % Number of cycles
pathFound=false;  % Whether the path was found
I_border = [];  % to indicate whether the point is out of border

counter_rand = 0;   % Number of sampling
rand_state = [];
if exist('RRT/rand_state.mat','file')
    load('RRT/rand_state.mat')
end
rng(1,'twister');

Time = 0;

while failedAttempts <= maxFailedAttempts
    tic();
    
    %% %%%%%%%%%%%%% visualize the obstacle obtained using NN %%%%%%%%%%%%%%
    %     ColliedPoints = [];
    %     ColliedJoints = [];
    % 	for numi = -0.95*pi : 0.01*pi : 0.95*pi
    %         for numj = -0.95*pi : 0.01*pi : 0.95*pi
    %             sample = [numi;numj];
    %             pos_enc = @(x)[x sin(x) cos(x)];
    %             obstacle_co = [4, 5, 3, 4; -7, -5, 5, 7; -8, 8, -10, -6];
    %             x_obstacle = [];
    %             for i = 1:3
    %                 x_span = obstacle_co(i,1) : 0.1 : obstacle_co(i,2);
    %                 y_span = obstacle_co(i,3) : 0.1 : obstacle_co(i,4);
    %                 [X_mg,Y_mg] = meshgrid(x_span, y_span);
    %                 x_obstacle = [x_obstacle, [X_mg(:) Y_mg(:)]'];
    %             end
    %             inp = pos_enc([repmat(flipud(sample)',[length(x_obstacle),1])'; x_obstacle]')';
    %             val = y_f(inp);
    %             if sum(sum(val<0.5))
    %                 % configuration space
    %                 figure(2)
    %                 hold on
    %                 plot(sample(1),sample(2),'*r');
    %                 ColliedJoints = [ColliedJoints; sample'];
    %                 hold off
    %
    %                 %task space
    %                 figure(3)
    %                 pos_enc = @(x)[x sin(x) cos(x)];
    %                 tmp = calc_fk(flipud(sample),r,d,alpha,base);
    %                 inp = pos_enc([flipud(sample)', tmp(3,1:2)])';
    %                 val = dy_f(inp);
    %
    %                 figure(3)
    %                 hold on
    %                 plot(tmp(3,1),tmp(3,2),'*r');
    %                 ColliedPoints = [ColliedPoints; tmp(3,1:2)];
    %                 drawnow
    %             end
    %             fprintf ('%d of %d\n', numi, numj);
    %         end
    %     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% %%%%%%%%%%%%%%%%%%% obtain P_new %%%%%%%%%%%%%%%%%%%
    
    if rand <= 1   % Sampling randomly or moving towards the target
        if ~ exist('RRT/rand_state.mat','file')
            sample = RandomSample();    %[q2;q1]
            rand_state = [rand_state; rng];
            counter_rand = counter_rand + 1;
        else
            sample = RandomSample();
            rng(rand_state(counter_rand + 1));
            counter_rand = counter_rand + 1;
        end
    else
        sample = end_coords;
        counter_rand = counter_rand + 1;
    end
    
    
    % Calculate the distance from sample to each nodes in RRT tree.
    
    % Here we assume that the Dist function can compute the
    % distance to multiple samples corresponding to the columns of
    % the second argument
    % at the end of this call the array distances will indicate the
    % distance between the new sample and each of the samples that has been
    % generated so far in the program.
    distances = Dist(sample, RRTree(1:dim, :));
    
    % Find the closest node.
    [M, I] = min(distances, [], 2);
    closestNode = RRTree(1:dim, I);
    closestNode_int = RRTree(end, I);
    
    % Calculate the new node
    if M > stepsize
        newPoint = closestNode + (sample-closestNode)/norm(sample-closestNode, 2) * stepsize;
    else
        newPoint = sample;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% %%%%%%%%%%%%%%%%%%% collision checking %%%%%%%%%%%%%%%%%%%
    
    % Chech wheter the path from closest node to new node is available
    [collied, colliedPoint, ObsPoint] = checkPath(closestNode, newPoint, obstacle, dim, NN_check, y_f);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% %%%%%%%%%%%%%%%%%%% if grad_heuristic = Ture, obtain a new p_new based on grad heuristic when collisied,
    %%%%%%%%%%%%%%%%%%% otherwise, discard it directly %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Time = Time + toc();
    
    if collied
        tic();
        
        %task space
        pos_enc = @(x)[x sin(x) cos(x)];
        tmp = calc_fk(flipud(colliedPoint),r,d,alpha,base);
        inp = pos_enc([flipud(colliedPoint)', ObsPoint'])';
        val = dy_f(inp);
        
        Time = Time + toc();
        
        if vis3
            figure(3)
            hold on
            %             plot(tmp(3,1),tmp(3,2),'*r');
            %             plot(ObsPoint(1),ObsPoint(2),'*b');
            %             direction = 10*([val(3),val(4)]+...
            %                 [val(7),val(8)] .* cos([ObsPoint(1),ObsPoint(2)])+...
            %                 [val(11),val(12)] .* (-sin([ObsPoint(1),ObsPoint(2)])))/3;
            %             Quiver_4 = quiver(ObsPoint(1),ObsPoint(2),direction(1),direction(2));
            %
            %             fv = ThreeLinkRobot (colliedPoint,dim);%[q2 q1]
            %             p = patch (fv);
            %             p.FaceColor = 'green';
            %             p.EdgeColor = 'none';
            %             p.FaceAlpha = 0.1;
            hold off
        end
        
        % joint space
        if vis2
            figure(2)
            hold on
            if ~isempty(Point_2)
                set(Point_2,'visible','off');
                set(Quiver_2,'visible','off');
            end
            Point_2 = plot(colliedPoint(1),colliedPoint(2),'*r');
            Joint = ([val(2),val(1)]+...
                [val(5),val(6)] .* cos(flipud(colliedPoint)')+...
                [val(9),val(10)] .* (-sin(flipud(colliedPoint)')))/3;
            if grad_heuristic
                Quiver_2 = quiver(colliedPoint(1),colliedPoint(2),Joint(2),Joint(1));
            end
            hold off
            
            %%%%%%%%%%%%%%%%%%%%%%%% Update newPoint is grad_heuristic = Ture %%%%%%%%%%%%%%%%%%%%%%%%
            if grad_heuristic & closestNode_int > 0
                q_prev = RRTree(1:dim, closestNode_int);    %[q2,q1]
                d_vector = closestNode - q_prev;    %[q2,q1]
                dGamma = [Joint(2),Joint(1)];   %[q2,q1]
                projection = (dot(dGamma', d_vector))/(dot(d_vector, d_vector))*d_vector;
                %                 projection = (dot(d_vector, dGamma'))/(dot(dGamma', dGamma'))*dGamma';  %[q2,q1]
                newPoint = closestNode + stepsize * (d_vector - projection)/norm(d_vector - projection);    %[q2,q1]
                
                hold on
                if ~isempty(Point_3)
                    set(Point_3,'visible','off');
                end
                %                     quiver(closestNode(1),closestNode(2),d_vector(1),d_vector(2));
                %                     quiver(closestNode(1),closestNode(2),projection(1),projection(2));
                Point_3 = plot(newPoint(1),newPoint(2),'or');
                hold off
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
        end
        
        % visualize grade
        if vis4
            figure(4)
            % moving the robot
            move_r(r_h,flipud(colliedPoint),r,d,alpha,base)
            
            x_span = linspace(q_min(3),q_max(3));
            y_span = linspace(q_min(4),q_max(4));
            [X_mg,Y_mg] = meshgrid(x_span, y_span);
            x = [X_mg(:) Y_mg(:)]';
            inp_4 = pos_enc([repmat(flipud(colliedPoint)',[length(x),1])'; x]')';
            val_4_y = y_f(inp_4);
            val_4_dy = dy_f(inp);
            hold on
            if ~isempty(Point_4)
                set(Point_4,'visible','off');
                set(Quiver_4,'visible','off');
            end
            
            direction = 10*([val_4_dy(3),val_4_dy(4)]+...
                [val_4_dy(7),val_4_dy(8)] .* cos([ObsPoint(1),ObsPoint(2)])+...
                [val_4_dy(11),val_4_dy(12)] .* (-sin([ObsPoint(1),ObsPoint(2)])))/3;
            if grad_heuristic
                Point_4 = plot(ObsPoint(1),ObsPoint(2),'*r');
                Quiver_4 = quiver(ObsPoint(1),ObsPoint(2),direction(1),direction(2));
            end
            hold off
            
            Z_mg = reshape(val_4_y,size(X_mg));
            cr1.ZData = Z_mg;
            ctr.ZData = Z_mg;
            ctr.LevelList=linspace(-3,15);
            
            % visualize dy_f by moving robot
            %             hold on
            %                 if ~isempty(r_h_dy)
            %                     set(r_h_dy,'visible','off');
            %                 end
            %                 r_h_dy = create_r(ax_h,flipud(colliedPoint) + 2/norm(val)*val(1:2),r,d,alpha,base);
            % %                 r_h_dy = create_r(ax_h,flipud(colliedPoint) + val(1:2)',r,d,alpha,base);
            %             hold off
        end
        
        failedAttempts = failedAttempts+1;
        continue;
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% %%%%%%%%%%%%%%%%%%% Add p_new to RRT Tree %%%%%%%%%%%%%%%%%%%
    tic();
    
    % Check whether the new node is near the target
    if Dist(newPoint,end_coords)<disTh
        pathFound=true;
        break;
    end
    
    % Check whether the new node is in the RRT tree
    [M, I2] = min(Dist(newPoint, RRTree(1:dim, :)), [], 2);
    if Dist(newPoint, RRTree(1:dim, I2)) < disTh
        failedAttempts = failedAttempts+1;
        continue;
    end
    
    Time = Time + toc();
    
    % RRT* or RRT
    if RRT_star
        tic();
        % search nodes in a circle of radius 'RadiusForNeib'
        disToNewList = [];
        nearIndexList = [];
        for index_near = 1:size(RRTree,2)
            disTonew = norm(newPoint - RRTree(1:dim,index_near));
            if(disTonew < RadiusForNeib)    % 满足条件:欧氏距离小于RadiusForNeib
                disToNewList = [disToNewList disTonew];     % 满足条件的所有节点到x_new的cost
                nearIndexList = [nearIndexList index_near];     % 满足条件的所有节点基于树T的索引
            end
        end
        
        temp_cost = RRTree(dim+1,I) + stepsize;
        temp_parent = I;
        x_mincost = closestNode;
        % select the parent of x_new to minmize the cost
        for cost_index = 1:length(nearIndexList)    % cost_index是基于disToNewList的索引,不是整棵树的索引
            costToNew = disToNewList(cost_index) + RRTree(dim+1,nearIndexList(cost_index));
            if costToNew * 1.00001 < temp_cost    % temp_cost为通过minDist节点的路径的cost
                x_mincost = RRTree(1:dim, nearIndexList(cost_index));   % 符合剪枝条件节点的坐标
                [collied, ~, ~] = checkPath(x_mincost, newPoint, obstacle, dim, NN_check, y_f);
                if collied
                    continue;   %有障碍物
                end
                temp_cost = costToNew;
                temp_parent = nearIndexList(cost_index);
            end
        end
        
        RRTree = [RRTree, [newPoint; temp_cost; temp_parent]];
        failedAttempts = 0;
        
        Time = Time + toc();
        
        % Visualize the expanding process of RRT
        if display
            if dim == 3
                %joint sapce
                figure(2)
                line([x_mincost(1);newPoint(1)],[closestNox_mincostde(2);newPoint(2)],[x_mincost(3);newPoint(3)],'color',[0 0 1]);
                drawnow
                
            elseif dim == 2
                %joint sapce
                if vis2
                    figure(2)
                    line([x_mincost(1);newPoint(1)],[x_mincost(2);newPoint(2)],'color',[0 0 1]);
                    drawnow
                end
                
                %task space
                if vis3
                    pos_enc = @(x)[x sin(x) cos(x)];
                    tmp_new = calc_fk(flipud(newPoint),r,d,alpha,base); % change [q2 q1] to [q1 q2]
                    tmp_close = calc_fk(flipud(x_mincost),r,d,alpha,base);
                    
                    figure(3)
                    hold on
                    line([tmp_close(3,1);tmp_new(3,1)],[tmp_close(3,2);tmp_new(3,2)],'color',[0 0 1]);
                    hold off
                    drawnow
                end
            end
        end
        
        % rewire
        for rewire_index = 1:length(nearIndexList)
            if nearIndexList(rewire_index) ~= temp_parent    % 若不是之前计算的最小cost的节点
                newCost = temp_cost + disToNewList(rewire_index);    % 计算neib经过x_new再到起点的代价
                if newCost < RRTree(dim+1, nearIndexList(rewire_index))    % 需要剪枝
                    x_neib = RRTree(1:dim, nearIndexList(rewire_index));
                    [collied, ~, ~] = checkPath(x_neib, newPoint, obstacle, dim, NN_check, y_f);
                    if collied
                        continue;   %有障碍物
                    end
                    RRTree(dim + 1, nearIndexList(rewire_index)) = newCost;
                    
                    Time = Time + toc();
                    
                    figure(2)
                    Parent_int = RRTree(end,nearIndexList(rewire_index));
                    
                    line([x_neib(1);newPoint(1)],[x_neib(2);newPoint(2)],'color',[0 1 0]);
                    line([x_neib(1);RRTree(1,Parent_int)],[x_neib(2);RRTree(2,Parent_int)],'color',[1 1 1]);
                    drawnow
                    
                    tic();
                    
                    RRTree(end, nearIndexList(rewire_index)) = size(RRTree,2);
                    
                end
            end
        end
        
        
        
    else
        tic();
        % Add the new node to the RRT tree
        RRTree = [RRTree, [newPoint; RRTree(dim+1,I) + stepsize ;I]];
        failedAttempts = 0;
        
        Time = Time + toc();
        
        % Visualize the expanding process of RRT
        if display
            if dim == 3
                %joint sapce
                figure(2)
                line([closestNode(1);newPoint(1)],[closestNode(2);newPoint(2)],[closestNode(3);newPoint(3)],'color',[0 0 1]);
                drawnow
                
            elseif dim == 2
                %joint sapce
                if vis2
                    figure(2)
                    line([closestNode(1);newPoint(1)],[closestNode(2);newPoint(2)],'color',[0 0 1]);
                    drawnow
                end
                
                %task space
                if vis3
                    pos_enc = @(x)[x sin(x) cos(x)];
                    tmp_new = calc_fk(flipud(newPoint),r,d,alpha,base); % change [q2 q1] to [q1 q2]
                    tmp_close = calc_fk(flipud(closestNode),r,d,alpha,base);
                    
                    figure(3)
                    hold on
                    line([tmp_close(3,1);tmp_new(3,1)],[tmp_close(3,2);tmp_new(3,2)],'color',[0 0 1]);
                    hold off
                    drawnow
                end
                % visualize grade
                %             figure(4)
                %             %moving the robot
                %             move_r(r_h,flipud(newPoint),r,d,alpha,base)
                %
                %             x_span = linspace(q_min(3),q_max(3));
                %             y_span = linspace(q_min(4),q_max(4));
                %             [X_mg,Y_mg] = meshgrid(x_span, y_span);
                %             x = [X_mg(:) Y_mg(:)]';
                %             inp = pos_enc([repmat(flipud(newPoint)',[length(x),1])'; x]')';
                %             val = y_f(inp);
                %
                %             Z_mg = reshape(val,size(X_mg));
                %             cr1.ZData = Z_mg;
                %             ctr.ZData = Z_mg;
                %             ctr.LevelList=linspace(-3,15);
            end
        end
        
    end
    %     counter = counter + 1;
end

% Add the path from final node to target
if display && pathFound
    if dim == 3
        line([closestNode(1);end_coords(1)],[closestNode(2);end_coords(2)],[closestNode(3);end_coords(3)]);
    elseif dim == 2
        line([closestNode(1);end_coords(1)],[closestNode(2);end_coords(2)]);
    end
end

% counter = counter + 1;

if pathFound
    disp('Found the path!');
elseif ~pathFound
    disp('No path found. maximum attempts reached');
    path = [];
end

if param.saveRand
    save RRT/rand_state rand_state
end

%% Visualize the found path
path = [end_coords];
prev = I;
while prev > 0
    path = [RRTree(1:dim, prev), path];
    if ~ismember(prev, I_border) & display == 1
        if dim == 3
            line(path(1,1:2),path(2,1:2),path(3,1:2),'color',[1 0 0],'linewidth',2);
        elseif dim ==2
            figure(2)
            line(path(1,1:2),path(2,1:2),'color',[1 0 0],'linewidth',2);
            
            pos_enc = @(x)[x sin(x) cos(x)];
            tmp_new = calc_fk(flipud(path(:,1)),r,d,alpha,base); % change [q2 q1] to [q1 q2]
            tmp_close = calc_fk(flipud(path(:,2)),r,d,alpha,base);
            
            figure(3)
            hold on
            line([tmp_close(3,1);tmp_new(3,1)],[tmp_close(3,2);tmp_new(3,2)],'color',[1 0 0],'linewidth',2);
            hold off
            drawnow
        end
    end
    prev = RRTree(end, prev);
end

% Calculate the length of planned path
pathLength = 0;
for i = 1:length(path)-1
    pathLength = pathLength + Dist(path(1:dim, i),path(1:dim, i+1));
    
end
fprintf('Runing time = %d \nNumber of iteration = %d \nNumber of expended nodes = %d \nPath Length = %d \n\n', Time, counter_rand,size(RRTree,2),pathLength);   % 打印运行时间toc和路径长度

if exist('results/result_0510.mat','file')
    load('results/result_0510.mat');
else
    result_0510 = [];
end
result_0510(end+1,:) = [RRT_star, NN_check, start_coords', end_coords', Time, counter_rand, size(RRTree,2), pathLength];

save results/result_0510 result_0510
if vis2
    saveas(figure(2),['results/cs_',num2str(start_coords(1)),'_',num2str(start_coords(2)),'_',num2str(end_coords(1)),'_',num2str(end_coords(2)),'_','RRTStar_',num2str(RRT_star),'_NNCheck_',num2str(NN_check),'.jpg']);
end
if vis3
    saveas(figure(3),['results/ts_',num2str(start_coords(1)),'_',num2str(start_coords(2)),'_',num2str(end_coords(1)),'_',num2str(end_coords(2)),'_','RRTStar_',num2str(RRT_star),'_NNCheck_',num2str(NN_check),'.jpg']);
end

end

function pts = calc_fk(j_state,r,d,alpha,base)
P = dh_fk(j_state,r,d,alpha,base);
pts = zeros(3,3);
for i = 1:1:3
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
