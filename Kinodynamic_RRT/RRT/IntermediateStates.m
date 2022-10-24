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
for t = 0.2 : 0.2 : newPos.t_1(1) + newPos.t_v(1) + newPos.t_2(1)
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
end
%%

% for i = 1 : size(q_list,2)
%     Time = Dist_kd(q_list(:,i), closestNode, cfg);
%     midPoint = Steer(closestNode, q_list(:,i), Time, cfg);
%     [collided, ~, ~] = checkPath_kd(closestNode, midPoint, obs, cfg.dim, cfg.NN_check, cfg.y_f, cfg.circle_obs, cfg);
%     if ~collided
%         RRTree = [RRTree, [q_list(:,i); RRTree(end-1,I) + Time ;I]];
%         %% Visualize
%         if (cfg.display2 && cfg.dim == 2) || cfg.display3
%             q_1_end = closestNode(1:2) + closestNode(3:4) .* midPoint.t_1 + 1/2 .* midPoint.a_1 .* midPoint.t_1.^2;
%             v_1_end = closestNode(3:4) + midPoint.a_1 .* midPoint.t_1;
%             q_v_end = q_1_end + v_1_end .* midPoint.t_v;
%             v_v_end = v_1_end;
%             q_2_end = q_v_end + v_v_end .* midPoint.t_2 + 1/2 * midPoint.a_2 .* midPoint.t_2.^2;
%             v_2_end = v_v_end + midPoint.a_2 .* midPoint.t_2;
%
%             q_list_vis = [];
%             i = 0;
%             for t = linspace(0, midPoint.t_1(1) + midPoint.t_v(1) + midPoint.t_2(1),100)
%                 i = i + 1;
%                 for j = 1:2
%                     if t < midPoint.t_1(j)
%                         q_1 = closestNode(j) + closestNode(j+2) * t + 1/2 * midPoint.a_1(j) * t^2;
% %                         v_1 = closestNode(j+2) + midPoint.a_1(j) * t;
%                     elseif t < midPoint.t_1(j) + midPoint.t_v(j)
%                         q_1 = q_1_end(j) + v_1_end(j) * (t - midPoint.t_1(j));
% %                         v_1 = v_1_end(j);
%                     else
%                         q_1 = q_v_end(j) + v_v_end(j) * (t - midPoint.t_v(j) - midPoint.t_1(j)) + ...
%                             1/2 * midPoint.a_2(j) * (t - midPoint.t_v(j) - midPoint.t_1(j))^2;
% %                         v_1 = v_v_end(j) + midPoint.a_2(j) * (t - midPoint.t_v(j) - midPoint.t_1(j));
%                     end
%                     q_list_vis(j,i) = q_1;
% %                     q_list_vis(j+2,i) = v_1;
%                 end
%             end
%             if cfg.display2 && cfg.dim == 2
%                 figure(handle.fig_handle2)
%                 hold on
%                 plot(handle.ax_h2, q_list_vis(2,:), q_list_vis(1,:),'r-');
% %                 plot(handle.ax_h2, q_list_vis(2,end), q_list_vis(1,end),'bo');
%                 hold off
%             end
%         end
%     end
% end
figure(handle.fig_handle2)
hold on
%                 plot(handle.ax_h2, q_list_vis(2,:), q_list_vis(1,:),'r-');
plot(handle.ax_h2, q_list_vis(2,end), q_list_vis(1,end),'bo');
hold off
end