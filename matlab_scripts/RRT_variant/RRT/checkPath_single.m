% 检测当前树结点到新结点的方向上，所有点在地图内部
function [feasible, dir, ObsPoint] = checkPath(dir, obstacle, dim, y_f)
feasible = false;
ObsPoint = [];

%% check by NN
pos_enc = @(x)[x sin(x) cos(x)];
obstacle_co = [4, 5, 3, 4; -7, -5, 5, 7; -8, 8, -10, -6];
x_obstacle = [];

% all area
%     for i = 1:3
%         x_span = obstacle_co(i,1) : 0.1 : obstacle_co(i,2);
%         y_span = obstacle_co(i,3) : 0.1 : obstacle_co(i,4);
%         [X_mg,Y_mg] = meshgrid(x_span, y_span);
%         x_obstacle = [x_obstacle, [X_mg(:) Y_mg(:)]'];
%     end

%just boundry
for i = 1:3
    x_span = obstacle_co(i,1) : 0.01 : obstacle_co(i,2);
    y_span = obstacle_co(i,3) : 0.01 : obstacle_co(i,4);
    l1 = [x_span', repmat(obstacle_co(i,3),length(x_span),1)];
    l2 = [x_span', repmat(obstacle_co(i,4),length(x_span),1)];
    l3 = [repmat(obstacle_co(i,1),length(y_span),1), y_span'];
    l4 = [repmat(obstacle_co(i,2),length(y_span),1), y_span'];
    x_obstacle = [x_obstacle, l1', l2', l3', l4'];
end

inp = pos_enc([repmat(flipud(dir)',[length(x_obstacle),1])'; x_obstacle]')';
val = y_f(inp);
if sum(sum(val<0.5))
    ObsPoint_int = find((val<0.5));
    ObsPoint = x_obstacle(:, ObsPoint_int(randperm(numel(ObsPoint_int),1)));
    feasible = true;
    return
end

%% check by traditional method
fv = ThreeLinkRobot (dir,dim);
if CollisionCheck(fv, obstacle)
    ObsPoint = dir;
    feasible = true;
    return
end
end