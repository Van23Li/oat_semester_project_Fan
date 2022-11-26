function x = RandomSample (obs,q_min,q_max,dim,cfg)
% Generate a random freespace configuration for the robot
% return [q1; q2]
while true
%     x = 360*rand(dim,1);
    x = q_min(1:dim)' + (q_max(1:dim) - q_min(1:dim))' .* rand(dim,1);
    if cfg.NN_check
        pos_enc = @(x)[x sin(x) cos(x)];
        if cfg.circle_obs
            inp = pos_enc([repmat(x', [size(obs.centers,1),1]), obs.centers])';
            val = cfg.y_f(inp);

            if ~sum(val<=obs.radiuses'+0.5)
                return
            end
        else
            obs_co = [4, 5, 3, 4; -7, -5, 5, 7; -8, 8, -10, -6];
            x_obs = [];
            % all area
            %     for i = 1:3
            %         x_span = obs_co(i,1) : 0.1 : obs_co(i,2);
            %         y_span = obs_co(i,3) : 0.1 : obs_co(i,4);
            %         [X_mg,Y_mg] = meshgrid(x_span, y_span);
            %         x_obs = [x_obs, [X_mg(:) Y_mg(:)]'];
            %     end
            %just boundry
            for i = 1:3
                x_span = obs_co(i,1) : 0.01 : obs_co(i,2);
                y_span = obs_co(i,3) : 0.01 : obs_co(i,4);
                l1 = [x_span', repmat(obs_co(i,3),length(x_span),1)];
                l2 = [x_span', repmat(obs_co(i,4),length(x_span),1)];
                l3 = [repmat(obs_co(i,1),length(y_span),1), y_span'];
                l4 = [repmat(obs_co(i,2),length(y_span),1), y_span'];
                x_obs = [x_obs, l1', l2', l3', l4'];
            end

            inp = pos_enc([repmat(x',[length(x_obs),1])'; x_obs]')';
            val = y_f(inp);
            if ~sum(sum(val<0.5))
                return
            end
        end
        
        
    else
    fv = ThreeLinkRobot (x,dim);
    
    if (~CollisionCheck(fv, obs))
        return
    end
    end
end