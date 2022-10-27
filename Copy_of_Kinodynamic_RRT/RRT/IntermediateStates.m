function RRTree = IntermediateStates(RRTree, closestNode, sample, newPos, I, T, obs, cfg, handle)
% I: closestNode's int
q_1_end = closestNode(1:2) + closestNode(3:4) .* newPos.t_1 + 1/2 .* newPos.a_1 .* newPos.t_1.^2;
v_1_end = closestNode(3:4) + newPos.a_1 .* newPos.t_1;
q_v_end = q_1_end + v_1_end .* newPos.t_v;
v_v_end = v_1_end;
q_2_end = q_v_end + v_v_end .* newPos.t_2 + 1/2 * newPos.a_2 .* newPos.t_2.^2;
v_2_end = v_v_end + newPos.a_2 .* newPos.t_2;

q_list = [];
i = 0;
for t = 0.2 : cfg.steptime : newPos.t_1(1) + newPos.t_v(1) + newPos.t_2(1)
    i = i + 1;
    for j = 1:2
        if t < newPos.t_1(j)
            q_1 = closestNode(j) + closestNode(j+2) * t + 1/2 * newPos.a_1(j) * t^2;
            v_1 = closestNode(j+2) + newPos.a_1(j) * t;
        elseif t < newPos.t_1(j) + newPos.t_v(j)
            q_1 = q_1_end(j) + v_1_end(j) * (t - newPos.t_1(j));
            v_1 = v_1_end(j);
        else
            q_1 = q_v_end(j) + v_v_end(j) * (t - newPos.t_v(j) - newPos.t_1(j)) + ...
                1/2 * newPos.a_2(j) * (t - newPos.t_v(j) - newPos.t_1(j))^2;
            v_1 = v_v_end(j) + newPos.a_2(j) * (t - newPos.t_v(j) - newPos.t_1(j));
        end
        q_list(j,i) = q_1;
        q_list(j+2,i) = v_1;
    end
end

RRTree = [RRTree, [sample; RRTree(end-1,I) + T ;I]];

%% Visualize
if (cfg.display2 && cfg.dim == 2) || cfg.display3
    q_list_vis = [];
    i = 0;
    for t = linspace(0, newPos.t_1(1) + newPos.t_v(1) + newPos.t_2(1),100)
        i = i + 1;
        for j = 1:2
            if t < newPos.t_1(j)
                q_1 = closestNode(j) + closestNode(j+2) * t + 1/2 * newPos.a_1(j) * t^2;
                %                 v_1 = closestNode(j+2) + newPos.a_1(j) * t;
            elseif t < newPos.t_1(j) + newPos.t_v(j)
                q_1 = q_1_end(j) + v_1_end(j) * (t - newPos.t_1(j));
                %                 v_1 = v_1_end(j);
            else
                q_1 = q_v_end(j) + v_v_end(j) * (t - newPos.t_v(j) - newPos.t_1(j)) + ...
                    1/2 * newPos.a_2(j) * (t - newPos.t_v(j) - newPos.t_1(j))^2;
                %                 v_1 = v_v_end(j) + newPos.a_2(j) * (t - newPos.t_v(j) - newPos.t_1(j));
            end
            q_list_vis(j,i) = q_1;
            %             q_list_vis(j+2,i) = v_1;
        end
    end
    if cfg.display2 && cfg.dim == 2
        figure(handle.fig_handle2)
        hold on
        plot(handle.ax_h2, q_list_vis(2,:), q_list_vis(1,:),'b-');
        plot(handle.ax_h2, q_list_vis(2,end), q_list_vis(1,end),'bo');
        hold off
    end
    
    
    if cfg.display3
        tmp_list = [];
        for i = 1:size(q_list_vis,2)
            tmp = calc_fk(q_list_vis(1:cfg.dim,i),cfg.r,cfg.d,cfg.alpha,cfg.base); % change [q2 q1] to [q1 q2]
            tmp_list = [tmp_list, tmp(end,1:2)'];
        end
        figure(handle.fig_handle3)
        hold on
        plot(handle.ax_h3, tmp_list(1,:), tmp_list(2,:),'b-');
        plot(handle.ax_h3, tmp_list(1,end), tmp_list(2,end),'bo');
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