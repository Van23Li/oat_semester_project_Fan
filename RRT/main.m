%% Draw Robot and obstacles
clear all
close all

% You should experiment by changing these coordinates
dim = 2;
if dim == 3
    start_coords = [300; 300; 20];
    end_coords = [200; 200; 200];
elseif dim ==2
    start_coords = [200; 20];
    end_coords = [50; 150];
end



figure(1);

subplot(1,2,1);

% This function sets up the two/three link robot based on the coordinates
% in configuration space [theta1, theta2, (theta3)]. You can change the configuration
% of the robot by changing the two numbers in the input array.
if dim == 3
    fv = ThreeLinkRobot ([30 -30 90],dim);
elseif dim == 2
    fv = ThreeLinkRobot ([-30 90],dim);
end
p = patch (fv);
p.FaceColor = 'blue';
p.EdgeColor = 'none';
sz = 12;
axis equal;
axis (sz*[-1 1 -1 1]);

% Add obstacles
hold on;
obstacle.vertices = [3 3; 3 4; 4 3; -6 6; -6 8; -7 6; -8 -6; 8 -6; -8 -10; 8 -10];
obstacle.faces = [1 2 3; 4 5 6; 7 8 9; 8 9 10];
obs = patch(obstacle);
hold off;

%% Compute Configuration Space
if dim == 3
    cuboidSize = [36, 36, 36];
    if ~exist('cspace_3.mat','file')
        theta1_range = 0:36:360;
        theta2_range = 0:36:360;
        theta3_range = 0:36:360;
        
        ndim1 = length(theta1_range);
        ndim2 = length(theta2_range);
        ndim3 = length(theta3_range);
        
        cspace = true(ndim1, ndim2, ndim3);
        
        for i = 1:ndim1
            for j = 1:ndim2
                for k = 1:ndim3
                    
                    fv = ThreeLinkRobot ([theta3_range(k) theta2_range(j) theta1_range(i)], dim);
                    
                    cspace (i,j,k) = CollisionCheck (fv, obstacle);
                    
                end
                
                fprintf ('%d of %d\n', i, ndim1);
            end
        end
        save cspace_3 cspace
    else
        load('cspace_3.mat')
    end
    %% Plot configuration space
    subplot (1,2,2);
    
    axis equal;
    axis ([0 360 0 360 0 360]);
    
    cmap = [1 1 1; 0 0 0];
    colormap(cmap);
    
    % Here we may flip the cspace image to match the axes
    for i = 1:length(cspace(1,:,:))
        for j = 1:length(cspace(2,:,:))
            for k = 1:length(cspace(3,:,:))
                
                if cspace(k,j,i) == 1
                    vertexIndex=[0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 1 0;1 1 1];
                    originPoint = [i, j, k];
                    vertex = cuboidSize .* (originPoint-1) + vertexIndex.*cuboidSize;
                    
                    facet=[1 2 4 3;1 2 6 5;1 3 7 5;2 4 8 6;3 4 8 7;5 6 8 7];
                    
                    patch('Vertices',vertex,'Faces',facet,'FaceColor','green', 'EdgeColor', [0.7,0.7,0.7]);
                    
                end
            end
        end
        axis xy;
        view(3);
        xlabel ('theta3 in degrees');
        ylabel ('theta2 in degrees');
        zlabel ('theta1 in degrees');
        
        title ('Configuration Space');
        
    end
elseif dim == 2
    
    if ~exist('cspace_2.mat','file')
        theta1_range = 0:10:360;
        theta2_range = 0:10:360;
        
        ndim1 = length(theta1_range);
        ndim2 = length(theta2_range);
        
        cspace = true(ndim1, ndim2);
        
        for i = 1:ndim1
            for j = 1:ndim2
                
                fv = ThreeLinkRobot ([theta2_range(j) theta1_range(i)], dim);
                
                cspace (i,j) = CollisionCheck (fv, obstacle);
                
            end
            
            fprintf ('%d of %d\n', i, ndim1);
        end
        save cspace_2 cspace
    else
        load('cspace_2.mat')
    end
    %% Plot configuration space
    subplot (1,2,2);
    
    axis equal;
    axis ([0 360 0 360]);
    
    cmap = [1 1 1; 0 0 0];
    colormap(cmap);
    
    % Here we may flip the cspace image to match the axes
    imagesc([0 360], [0 360], cspace);
    axis xy;
    
    xlabel ('theta2 in degrees');
    ylabel ('theta1 in degrees');
    
    title ('Configuration Space');
end

%% Plot a path through torus space
figure(2);
axis equal;
axis ([0 360 0 360 0 360]);
if dim == 3
    for i = 1:length(cspace(1,:,:))
        for j = 1:length(cspace(2,:,:))
            for k = 1:length(cspace(3,:,:))
                
                if cspace(k,j,i) == 1
                    vertexIndex=[0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 1 0;1 1 1];
                    originPoint = [i, j, k];
                    vertex = cuboidSize .* (originPoint-1) + vertexIndex.*cuboidSize;
                    
                    facet=[1 2 4 3;1 2 6 5;1 3 7 5;2 4 8 6;3 4 8 7;5 6 8 7];
                    
                    patch('Vertices',vertex,'Faces',facet,'FaceColor','green', 'EdgeColor', [0.7,0.7,0.7]);
                    
                end
            end
        end
        axis xy;
        view(3);
        xlabel ('theta3 in degrees');
        ylabel ('theta2 in degrees');
        zlabel ('theta1 in degrees');
        
        title ('Configuration Space');
        
    end
elseif dim == 2
    cmap = [1 1 1; 0 0 0];
    colormap(cmap);
    
    % Here we may flip the cspace image to match the axes
    imagesc([0 360], [0 360], cspace);
    axis xy;
    
    xlabel ('theta2 in degrees');
    ylabel ('theta1 in degrees');
    
    title ('Configuration Space');
end

%% Plan a path
% Visualize the starting and end nodes
if dim == 3
    hold on
    plot3(start_coords(1),start_coords(2),start_coords(3),'gp');
    plot3(end_coords(1),end_coords(2),end_coords(3),'rp');
    hold off
elseif dim ==2
    hold on
    plot(start_coords(1),start_coords(2),'gp');
    plot(end_coords(1),end_coords(2),'rp');
    hold off
end

vertex_coords=[start_coords, end_coords];

path = RRT(@()(RandomSampleThreeLink(obstacle,dim)), @DistThreeLink, ...
    obstacle, vertex_coords, dim, true, 10, 10, 10000);
