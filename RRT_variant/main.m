%% Draw Robot and obstacles
addpath('RRT')
clear all
close all

k1 = 0.95; k2 = 0.95;
param.q_min = [-k1*pi, -k2*pi, -10,-10];
param.q_max = [k1*pi, k2*pi, 10, 10];
param.q_init = [ -2, -1, 0, 0];

% % You should experiment by changing these coordinates
% param.dim = 2;
% if param.dim == 3
%     start_coords = [300*pi/180; 300*pi/180; 20*pi/180];
%     end_coords = [200*pi/180; 200*pi/180; 200*pi/180];
% elseif param.dim ==2
%     start_coords = fliplr(param.q_init(1:param.dim))';
%    end_coords = [-2; -2];
% %     end_coords = [0; 2.8];
% end

while(1)
    param.dim = 2;
    
    % Add obstacles
    hold on;
    obstacle.vertices = [4 3; 4 4; 5 3; 5 4 ; -5 5; -5 7; -7 5; -7 7; -8 -6; 8 -6; -8 -10; 8 -10];
    obstacle.faces = [1 2 3; 2 3 4 ; 5 6 7; 6 7 8; 9 10 11; 10 11 12];
    obs = patch(obstacle);
    hold off;
    
    %gmm model creation
    load('data/net50_pos_thr.mat');
    [y_f, dy_f] = tanhNN(net);
    
    % Add start and ending pose
    collied1 = 1;
    collied2 = 1;
    while collied1 | collied2
        if param.dim ==2
            start_coords = [param.q_min(1) + (param.q_max(1)-param.q_min(1))*rand; ...
                param.q_min(2) + (param.q_max(2)-param.q_min(2))*rand];
            end_coords   = [param.q_min(1) + (param.q_max(1)-param.q_min(1))*rand; ...
                param.q_min(2) + (param.q_max(2)-param.q_min(2))*rand];
        end
        [collied1, colliedPoint, ObsPoint] = checkPath_single(start_coords, obstacle, param.dim, y_f);
        [collied2, colliedPoint, ObsPoint] = checkPath_single(end_coords, obstacle, param.dim, y_f);
    end
    
    for RRT_star = 0
        param.RRT_star = RRT_star;
        for NN_check = 1
            close all

            param.NN_check = NN_check;
            if param.NN_check
                param.grad_heuristic = 1;
            else
                param.grad_heuristic = 0;
            end
            
            
            figure(1);
            
            subplot(1,2,1);
            
            % This function sets up the two/three link robot based on the coordinates
            % in configuration space [theta1, theta2, (theta3)]. You can change the configuration
            % of the robot by changing the two numbers in the input array.
            if param.dim == 3
                fv = ThreeLinkRobot ([30*pi/180 -30*pi/180 90*pi/180],param.dim);
            elseif param.dim == 2
                fv = ThreeLinkRobot (fliplr(param.q_init(1:param.dim)),param.dim);%[q2 q1]
            end
            p = patch (fv);
            p.FaceColor = 'blue';
            p.EdgeColor = 'none';
            sz = 12;
            axis equal;
            axis (sz*[-1 1 -1 1]);
            
            
            
            %% Compute Configuration Space
            if param.dim == 3
                cuboidSize = [36, 36, 36];
                if ~exist('RRT/cspace_3.mat','file')
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
                                
                                fv = ThreeLinkRobot ([theta3_range(k) theta2_range(j) theta1_range(i)], param.dim);
                                
                                cspace (i,j,k) = CollisionCheck (fv, obstacle);
                                
                            end
                            
                            fprintf ('%d of %d\n', i, ndim1);
                        end
                    end
                    save RRT/cspace_3 cspace
                else
                    load('RRT/cspace_3.mat')
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
            elseif param.dim == 2
                
                if ~exist('RRT/cspace_2.mat','file')
                    theta1_range = param.q_min(1):0.01*pi:param.q_max(1);
                    theta2_range = param.q_min(2):0.01:param.q_max(2);
                    
                    ndim1 = length(theta1_range);
                    ndim2 = length(theta2_range);
                    
                    cspace = true(ndim1, ndim2);
                    
                    for i = 1:ndim1
                        for j = 1:ndim2
                            
                            fv = ThreeLinkRobot ([theta2_range(j) theta1_range(i)], param.dim);
                            
                            cspace (i,j) = CollisionCheck (fv, obstacle);
                            
                        end
                        
                        fprintf ('%d of %d\n', i, ndim1);
                    end
                    save RRT/cspace_2 cspace
                else
                    load('RRT/cspace_2.mat')
                end
                %% Plot configuration space
                subplot (1,2,2);
                
                axis equal;
                axis ([param.q_min(2) param.q_max(2) param.q_min(1) param.q_max(1)]);
                
                cmap = [1 1 1; 0 0 0];
                colormap(cmap);
                
                % Here we may flip the cspace image to match the axes
                imagesc([param.q_min(2) param.q_max(2)], [param.q_min(1) param.q_max(1)], cspace);
                axis xy;
                
                xlabel ('theta2 in degrees');
                ylabel ('theta1 in degrees');
                
                title ('Configuration Space');
            end
            
            %% Plot a path through torus space
            figure('name','configuration space (RRT)','Position',[600 10 450 353]);
            axis equal;
            axis ([param.q_min(2) param.q_max(2), param.q_min(1) param.q_max(1)]);
            if param.dim == 3
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
            elseif param.dim == 2
                cmap = [1 1 1; 0 0 0];
                colormap(cmap);
                
                % Here we may flip the cspace image to match the axes
                imagesc([param.q_min(2) param.q_max(2)], [param.q_min(1) param.q_max(1)], cspace);
                axis xy;
                
                xlabel ('theta2 in degrees');
                ylabel ('theta1 in degrees');
                
                title ('Configuration Space');
            end
            
            %% Plan a path
            % Visualize the starting and end nodes
            if param.dim == 3
                hold on
                plot3(start_coords(1),start_coords(2),start_coords(3),'gp');
                plot3(end_coords(1),end_coords(2),end_coords(3),'rp');
                hold off
            elseif param.dim ==2
                hold on
                plot(start_coords(1),start_coords(2),'gp');
                plot(end_coords(1),end_coords(2),'rp');
                hold off
            end
            
            param.vertex_coords=[start_coords, end_coords];
            
            % add a new figure to visualize task space
            figure('name','task space','Position',[1000 600 450 353]);
            % if param.dim == 3
            %     fv = ThreeLinkRobot ([30*pi/180 -30*pi/180 90*pi/180],param.dim);
            % elseif param.dim == 2
            %     fv = ThreeLinkRobot ([-30*pi/180 90*pi/180],param.dim);
            % end
            % p = patch (fv);
            % p.FaceColor = 'blue';
            % p.EdgeColor = 'none';
            sz = 12;
            axis equal;
            axis (sz*[-1 1 -1 1]);
            % Add obstacles
            hold on;
            obs = patch(obstacle);
            hold off;
            
            param.display = true;
            param.stepsize = 0.1;
            param.disTh = 0.1;
            param.maxFailedAttempts = 10000;
            param.saveRand = false;
            
            fprintf('RRT_star = %d \nNN_check = %d \n\n', param.RRT_star,param.NN_check);   % 打印运行时间toc和路径长度
            path = RRT(@()(RandomSampleThreeLink(obstacle,param.q_min,param.q_max,param.dim)), @DistThreeLink, ...
                obstacle, param);
        end
    end
end