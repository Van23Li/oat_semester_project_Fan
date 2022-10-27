% 检测当前树结点到新结点的方向上，所有点在地图内部
function [collided, dir, ObsPoint] = checkPath(n, newPos, obs, dim, NN_check, y_f, circle_obs)
collided = false;
dir = [];
ObsPoint = [];
if NN_check
    pos_enc = @(x)[x sin(x) cos(x)];
    if circle_obs
%         for r = [norm(n - newPos, 2): -0.02: 0]
        for r = [0 : 0.04: norm(n - newPos, 2)]
            dir = n + (newPos - n)/norm(n-newPos, 2) * r;
            inp = pos_enc([repmat(dir', [size(obs.centers,1),1]), obs.centers])';
            val = y_f(inp);
            
            if sum(val<=obs.radiuses'+0.5)
                collided = true;
                ObsPoint = obs.centers(find(val == min(val)),:)';
                return
            end
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