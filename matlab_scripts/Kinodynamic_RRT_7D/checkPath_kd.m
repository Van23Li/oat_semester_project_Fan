% 检测当前树结点到新结点的方向上，所有点在地图内部
function [collided, dir, ObsPoint] = checkPath_kd(n, newPos, obs, dim, NN_check, y_f, circle_obs, cfg)
% newPos include [a_1, v_limit, a_2, t_1, t_v, t_2]
collided = false;
dir = [];
ObsPoint = [];
if NN_check
    pos_enc = @(x)[x sin(x) cos(x)];
    if circle_obs
        q_1_end = n(1:cfg.dim) + n(cfg.dim + 1:2 * cfg.dim) .* newPos.t_1 + 1/2 .* newPos.a_1 .* newPos.t_1.^2;
        v_1_end = n(cfg.dim + 1:2 * cfg.dim) + newPos.a_1 .* newPos.t_1;
        q_v_end = q_1_end + v_1_end .* newPos.t_v;
        v_v_end = v_1_end;
        q_2_end = q_v_end + v_v_end .* newPos.t_2 + 1/2 * newPos.a_2 .* newPos.t_2.^2;
        v_2_end = v_v_end + newPos.a_2 .* newPos.t_2;
        
        q_list = [];
        i = 0;
%         for t = linspace(0, newPos.t_1(1) + newPos.t_v(1) + newPos.t_2(1), 10)
        for t = 0 : 0.05 : newPos.t_1(1) + newPos.t_v(1) + newPos.t_2(1)
            i = i + 1;
            for j = 1:cfg.dim
                if t < newPos.t_1(j)
                    q_1 = n(j) + n(j+cfg.dim) * t + 1/2 * newPos.a_1(j) * t^2;
                elseif t < newPos.t_1(j) + newPos.t_v(j)
                    q_1 = q_1_end(j) + v_1_end(j) * (t - newPos.t_1(j));
                else
                    q_1 = q_v_end(j) + v_v_end(j) * (t - newPos.t_v(j) - newPos.t_1(j)) + ...
                           1/2 * newPos.a_2(j) * (t - newPos.t_v(j) - newPos.t_1(j))^2;
                end
                q_list(j,i) = q_1;
            end
            dir = q_list(:, end);
            inp = pos_enc([repmat(dir', [size(obs.centers,1),1]), obs.centers])';
            val = y_f(inp);
            
            if sum(val<=obs.radiuses'+0.5) || sum(dir <= cfg.q_max(1:cfg.dim)') ~= length(dir) || ...
                                sum(dir >= cfg.q_min(1:cfg.dim)') ~= length(dir)
                collided = true;
                ObsPoint = obs.centers(find(val == min(val)),:)';
                return
            end
        end
%         figure(5)
%         plot(q_list(2,:),q_list(1,:),'-');
%         axis([-2.5 2.5 -2.5 2.5]);
%         a=2
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
        
        
        for r = [norm(n - newPos, 2): -0.01: 0]
            %     for r = linspace(0, norm(n - newPos, 2), 10)
            dir = n + (newPos - n)/norm(n - newPos, 2) * r;
            inp = pos_enc([repmat(dir',[length(x_obs),1])'; x_obs]')';
            val = y_f(inp);
            if sum(sum(val<0.5))
                ObsPoint_int = find((val<0.5));
                ObsPoint = x_obs(:, ObsPoint_int(randperm(numel(ObsPoint_int),1)));
                collided = true;
                break;
            end
        end
    end
else
    for r = [norm(n - newPos, 2): -0.01: 0]
        dir = n + (newPos - n)/norm(n-newPos, 2) * r;
        fv = ThreeLinkRobot (flipud(dir),dim);
        if CollisionCheck(fv, obs)
            ObsPoint = dir;
            collided = true;
            break;
        end
    end
end
end