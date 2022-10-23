function handle = visual_init(obs, cfg, y_f)
%% Visualize collision
if cfg.display1
    %% Figure with robot and obstacles
    fig_handle = figure('Name','Two-dimensional robot','Position',[600 500 400 400]);
    ax_h = axes(fig_handle,'View',[0 90]);
    axis equal
    hold on
    ax_h.XLim = [cfg.q_min(end - 1) cfg.q_max(end - 1)];
    ax_h.YLim = [cfg.q_min(end) cfg.q_max(end)];
    ax_h.ZLim = [-0.1 0.1];
    
    %create contour object
    x_span = linspace(cfg.q_min(end - 1),cfg.q_max(end - 1));
    y_span = linspace(cfg.q_min(end),cfg.q_max(end));
    [X_mg,Y_mg] = meshgrid(x_span, y_span);
    [~, ctr] = contourf(ax_h, X_mg,Y_mg,0*X_mg,100,'LineStyle','none');
    lvl = 0.5;
    [~, cr1] = contour(ax_h, X_mg,Y_mg,X_mg-10,[lvl,lvl+0.001],'LineStyle','-','LineColor','k','LineWidth',2);
    
    % Plot obstractles
    if cfg.circle_obs
        for i = 1 : size(obs.centers,1)
            t = 0 : 0.1 : 2 * pi;
            x = obs.radiuses(i) * cos(t) + obs.centers(i,1);
            y = obs.radiuses(i) * sin(t) + obs.centers(i,2);
            
            patch(x, y,'k','facealpha', 0.4,'edgecolor','none');
        end
    else
        patch(obs,'FaceAlpha',0.4,'EdgeColor','none','LineStyle','none');
    end
    
    % Plot robot
    r_h = create_r(ax_h,cfg.start_coords, cfg.r, cfg.d, cfg.alpha, cfg.base);
    hold off
    
    %output
    handle.fig_handle = fig_handle;
    handle.ax_h = ax_h;
    handle.r_h = r_h;
    handle.ctr = ctr;
    handle.cr1 = cr1;
end

%% Plot configuration space
if cfg.display2 & cfg.dim == 2
    fig_handle2 = figure('Name','Configuration space','Position',[1000 500 400 400]);
    ax_h2 = axes(fig_handle2,'View',[0 90]);
    axis equal
    ax_h2.XLim = [cfg.q_min(2), cfg.q_max(2)];
    ax_h2.YLim = [cfg.q_min(1), cfg.q_max(1)];
    
    if cfg.NN_check == 1
        if cfg.circle_obs
            % Compute Configuration Space
            if ~exist('RRT/cspace_2_NN_circle.mat','file')
                cspace = zeros(size(cfg.q_min(1) : 0.001*pi : cfg.q_max(1),2),...
                    size(cfg.q_min(2) : 0.001*pi : cfg.q_max(2),2));
                pos_enc = @(x)[x sin(x) cos(x)];
                ii = 0;
                for i = cfg.q_min(1) : 0.001*pi : cfg.q_max(1)
                    ii = ii + 1; jj = 0;
                    for j = cfg.q_min(2) : 0.001*pi : cfg.q_max(2)
                        jj = jj + 1;
                        inp = pos_enc([repmat([i;j]',[size(obs.centers,1),1]), obs.centers])';
                        val = y_f(inp);
                        if sum(val<=obs.radiuses'+0.5)
                            cspace(ii, jj) = 1;
                        end
                    end
                    fprintf ('%d of %d\n', i, cfg.q_max(2));
                end
                save RRT/cspace_2_NN_circle cspace
            else
                load('RRT/cspace_2_NN_circle.mat')
            end
            
            cmap = [1 1 1; 0 0 0];
            colormap(cmap);
            
            % Here we may flip the cspace image to match the axes
            imagesc(ax_h2, [cfg.q_min(2) cfg.q_max(2)], [cfg.q_min(1) cfg.q_max(1)], cspace);
            axis equal
            ax_h2.XLim = [cfg.q_min(2), cfg.q_max(2)];
            ax_h2.YLim = [cfg.q_min(1), cfg.q_max(1)];
            axis xy;
            xlabel ('theta2 in degrees');
            ylabel ('theta1 in degrees');
            title ('Configuration Space');
        else
            % Compute Configuration Space
            if ~exist('RRT/cspace_2_NN.mat','file')
                
                cspace = zeros(size(cfg.q_min(1) : 0.01*pi : cfg.q_max(1),2),...
                    size(cfg.q_min(2) : 0.01*pi : cfg.q_max(2),2));
                pos_enc = @(x)[x sin(x) cos(x)];
                
                obstacle_co = [4, 5, 3, 4; -7, -5, 5, 7; -8, 8, -10, -6];
                x_obstacle = [];
                for i = 1:3
                    x_span = obstacle_co(i,1) : 0.1 : obstacle_co(i,2);
                    y_span = obstacle_co(i,3) : 0.1 : obstacle_co(i,4);
                    [X_mg,Y_mg] = meshgrid(x_span, y_span);
                    x_obstacle = [x_obstacle, [X_mg(:) Y_mg(:)]'];
                end
                
                ii = 0;
                for i = cfg.q_min(1) : 0.01*pi : cfg.q_max(1)
                    ii = ii + 1; jj = 0;
                    fprintf ('%d of %d\n', i, cfg.q_max(2));
                    for j = cfg.q_min(2) : 0.01*pi : cfg.q_max(2)
                        jj = jj + 1;
                        inp = pos_enc([repmat([i;j]',[length(x_obstacle),1])'; x_obstacle]')';
                        val = y_f(inp);
                        if sum(sum(val<0.5))
                            cspace(ii, jj) = 1;
                        end
                    end
                end
                save RRT/cspace_2_NN cspace
            else
                load('RRT/cspace_2_NN.mat')
            end
            
            cmap = [1 1 1; 0 0 0];
            colormap(cmap);
            
            % Here we may flip the cspace image to match the axes
            imagesc(ax_h2, [cfg.q_min(2) cfg.q_max(2)], [cfg.q_min(1) cfg.q_max(1)], cspace);
            axis equal
            ax_h2.XLim = [cfg.q_min(2), cfg.q_max(2)];
            ax_h2.YLim = [cfg.q_min(1), cfg.q_max(1)];
            axis xy;
            xlabel ('theta2 in degrees');
            ylabel ('theta1 in degrees');
            title ('Configuration Space');
        end
    else
        % Compute Configuration Space
        if ~exist('RRT/cspace_2_trad.mat','file')
            theta1_range = cfg.q_min(1):0.1*pi:cfg.q_max(1);
            theta2_range = cfg.q_min(2):0.1:cfg.q_max(2);
            
            ndim1 = length(theta1_range);
            ndim2 = length(theta2_range);
            
            cspace = true(ndim1, ndim2);
            
            for i = 1:ndim1
                for j = 1:ndim2
                    
                    fv = ThreeLinkRobot ([theta2_range(j) theta1_range(i)], cfg.dim);
                    
                    cspace (i,j) = CollisionCheck (fv, obs);
                    
                end
                
                fprintf ('%d of %d\n', i, ndim1);
            end
            save RRT/cspace_2_trad cspace
        else
            load('RRT/cspace_2_trad.mat')
        end
        
        cmap = [1 1 1; 0 0 0];
        colormap(cmap);
        
        % Here we may flip the cspace image to match the axes
        imagesc(ax_h2, [cfg.q_min(2) cfg.q_max(2)], [cfg.q_min(1) cfg.q_max(1)], cspace);
        axis equal
        ax_h2.XLim = [cfg.q_min(2), cfg.q_max(2)];
        ax_h2.YLim = [cfg.q_min(1), cfg.q_max(1)];
        axis xy;
        xlabel ('theta2 in degrees');
        ylabel ('theta1 in degrees');
        title ('Configuration Space');
    end
    
    %% Visualize the starting and end nodes
    figure(fig_handle2)
    hold on
    plot(ax_h2, cfg.start_coords(2),cfg.start_coords(1),'gp');  % start_coords: [q2;q1]
    plot(ax_h2, cfg.end_coords(2),cfg.end_coords(1),'rp');
    hold off
    
    
    %output
    handle.fig_handle2 = fig_handle2;
    handle.ax_h2 = ax_h2;
end

%% Plot task space
if cfg.display3
    %% Figure with robot and obstacles
    fig_handle3 = figure('Name','Two-dimensional robot','Position',[1000 10 400 400]);
    ax_h3 = axes(fig_handle3,'View',[0 90]);
    axis equal
    hold on
    ax_h3.XLim = [cfg.q_min(end - 1) cfg.q_max(end - 1)];
    ax_h3.YLim = [cfg.q_min(end) cfg.q_max(end)];
    ax_h3.ZLim = [-0.1 0.1];
    
    % Plot obstractles
    if cfg.circle_obs
        for i = 1 : size(obs.centers,1)
            t = 0 : 0.1 : 2 * pi;
            x = obs.radiuses(i) * cos(t) + obs.centers(i,1);
            y = obs.radiuses(i) * sin(t) + obs.centers(i,2);
            
            patch(x, y,'k','facealpha', 0.4,'edgecolor','none');
        end
    else
        patch(obs,'FaceAlpha',0.4,'EdgeColor','none','LineStyle','none');
    end
    
    % Plot robot
    create_r(ax_h3,cfg.start_coords, cfg.r, cfg.d, cfg.alpha, cfg.base);
    hold off
    
    %% Visualize the starting and end nodes
    hold on
    start_pose = calc_fk(cfg.start_coords,cfg.r,cfg.d,cfg.alpha,cfg.base);
    end_pose = calc_fk(cfg.end_coords,cfg.r,cfg.d,cfg.alpha,cfg.base);
    plot(ax_h3, start_pose(end,1), start_pose(end,2),'gp');  % start_coords: [q2;q1]
    plot(ax_h3, end_pose(end,1), end_pose(end,2),'rp');
    
    pts = calc_fk(cfg.end_coords,cfg.r,cfg.d,cfg.alpha,cfg.base);
    plot(ax_h3, pts(:,1),pts(:,2),'LineWidth',2,...
        'Marker','o','MarkerFaceColor','k','MarkerSize',4, 'Color', [1,0,0]);
    
    
    hold off
    
    %output
    handle.fig_handle3 = fig_handle3;
    handle.ax_h3 = ax_h3;
end

%% Some functions
    function handle = create_r(ax_h,j_state,r,d,alpha,base)
        pts = calc_fk(j_state,r,d,alpha,base);
        handle = plot(ax_h, pts(:,1),pts(:,2),'LineWidth',2,...
            'Marker','o','MarkerFaceColor','k','MarkerSize',4, 'Color', [0,0,1])
    end

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
end