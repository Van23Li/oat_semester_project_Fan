function t_end = plot_qdot(node1, node2, cfg, q_int, t_end, ax_handle)

Time = Dist_kd(node2, node1, cfg);
finalPoint = Steer(node1, node2, Time, cfg);


q_1_end = node1(1:cfg.dim) + node1(cfg.dim + 1:2 * cfg.dim) .* finalPoint.t_1 + 1/2 .* finalPoint.a_1 .* finalPoint.t_1.^2;
v_1_end = node1(cfg.dim + 1:2 * cfg.dim) + finalPoint.a_1 .* finalPoint.t_1;
q_v_end = q_1_end + v_1_end .* finalPoint.t_v;
v_v_end = v_1_end;
q_2_end = q_v_end + v_v_end .* finalPoint.t_2 + 1/2 * finalPoint.a_2 .* finalPoint.t_2.^2;
v_2_end = v_v_end + finalPoint.a_2 .* finalPoint.t_2;

q_list_vis = [];
t_list_vis = [];
i = 0;
for t = linspace(0, finalPoint.t_1(1) + finalPoint.t_v(1) + finalPoint.t_2(1),100)
    i = i + 1;
%     for j = 1:2
        if t < finalPoint.t_1(q_int)
            q_1 = node1(q_int) + node1(q_int+cfg.dim) * t + 1/2 * finalPoint.a_1(q_int) * t^2;
        elseif t < finalPoint.t_1(q_int) + finalPoint.t_v(q_int)
            q_1 = q_1_end(q_int) + v_1_end(q_int) * (t - finalPoint.t_1(q_int));
        else
            q_1 = q_v_end(q_int) + v_v_end(q_int) * (t - finalPoint.t_v(q_int) - finalPoint.t_1(q_int)) + ...
                1/2 * finalPoint.a_2(q_int) * (t - finalPoint.t_v(q_int) - finalPoint.t_1(q_int))^2;
        end
        q_list_vis(1,i) = t + t_end;
        q_list_vis(2,i) = q_1;
%     end
end

if cfg.display5
    hold on
    index = find(q_list_vis(1,:)<finalPoint.t_1(q_int) + t_end);
    plot(ax_handle, q_list_vis(1,index), q_list_vis(2,index),'LineStyle','-','color',[1 0 0]);
    index = find(q_list_vis(1,:)<=finalPoint.t_1(q_int) + finalPoint.t_v(q_int) + t_end & q_list_vis(1,:)>finalPoint.t_1(q_int) + t_end);
    plot(ax_handle, q_list_vis(1,index), q_list_vis(2,index),'LineStyle','-','color',[0 1 0]);
    index = find(q_list_vis(1,:)>finalPoint.t_1(q_int) + finalPoint.t_v(q_int) + t_end);
    plot(ax_handle, q_list_vis(1,index), q_list_vis(2,index),'LineStyle','-','color',[0 0 1]);
    hold off
end
t_end = finalPoint.t_1(1) + finalPoint.t_v(1) + finalPoint.t_2(1) + t_end;
end