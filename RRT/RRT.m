function path = RRT (RandomSample, Dist, obstacle, vertex_coords, dim, display, stepsize, disTh, maxFailedAttempts)
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
%
% Output :
%    path : An array containing the linear indices of the cells along the
%    shortest path from start to dest or an empty array if there is no
%    path.

tic;
start_coords = vertex_coords(:, 1);
end_coords = vertex_coords(:, 2);

RRTree = double([start_coords; -1]); % The index of the starting point is set to - 1
failedAttempts = 0;  % Number of failures
counter = 0;  % Number of cycles
pathFound=false;  % Whether the path was found
I_border = [];  % to indicate whether the point is out of border

while failedAttempts <= maxFailedAttempts
    if rand <= 1   % Sampling randomly or moving towards the target
        sample = RandomSample();
    else
        sample = end_coords;
    end
    
    % Calculate the distance from sample to each nodes in RRT tree.
    
    % Here we assume that the Dist function can compute the
    % distance to multiple samples corresponding to the columns of
    % the second argument
    % at the end of this call the array distances will indicate the
    % distance between the new sample and each of the samples that has been
    % generated so far in the program.
    [distances, I_min] = Dist(sample, RRTree(1:dim, :));
    
    % Find the closest node.
    [M, I] = min(distances, [], 2);
    closestNode = RRTree(1:dim, I);
    
    % Calculate the new node
    if M > stepsize
        newPoint = closestNode + I_min(:,I).*(sample-closestNode)/norm(sample-closestNode, 2) * stepsize;
    else
        newPoint = sample;
    end
    newPoint_border = [newPoint < 0, newPoint > 360];
    if sum(sum(newPoint_border)) > 0
        I_border = [I_border; I];
    end
    
    % Chech wheter the path from closest node to new node is available
    if ~checkPath(closestNode, newPoint, obstacle, dim)
        failedAttempts = failedAttempts+1;
        continue;
    end
    
    newPoint = mod(newPoint, 360);
    
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
    
    % Add the new node to the RRT tree
    RRTree = [RRTree, [newPoint; I]];
    failedAttempts = 0;
    
    % Visualize the expanding process of RRT
    if display & sum(sum(newPoint_border))==0
        if dim == 3
            line([closestNode(1);newPoint(1)],[closestNode(2);newPoint(2)],[closestNode(3);newPoint(3)],'color',[0 0 1]);
            drawnow
        elseif dim == 2
            line([closestNode(1);newPoint(1)],[closestNode(2);newPoint(2)],'color',[0 0 1]);
            drawnow
        end
    end
    counter = counter + 1;
end
% Add the path from final node to target
if display && pathFound
    if dim == 3
        line([closestNode(1);end_coords(1)],[closestNode(2);end_coords(2)],[closestNode(3);end_coords(3)]);
    elseif dim == 2
        line([closestNode(1);end_coords(1)],[closestNode(2);end_coords(2)]);
    end
end
counter=counter+1;

if pathFound
    disp('Found the path!');
elseif ~pathFound
    disp('No path found. maximum attempts reached');
    path = [];
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
            line(path(1,1:2),path(2,1:2),'color',[1 0 0],'linewidth',2);
        end
    end
    prev = RRTree(end, prev);
end

% Calculate the length of planned path
pathLength = 0;
for i = 1:length(path)-1
    pathLength = pathLength + Dist(path(1:dim, i),path(1:dim, i+1));
end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength);   % 打印运行时间toc和路径长度
