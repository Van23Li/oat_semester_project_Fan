% function plot_anime(handle.ax_h3,path(1:cfg.dim,i), cfg.r, cfg.d, cfg.alpha, cfg.base)
function q_list_vis = plot_anime(node1, node2, cfg)
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
% for t = linspace(0, finalPoint.t_1(1) + finalPoint.t_v(1) + finalPoint.t_2(1),10)
for t = [0 : 0.1 : finalPoint.t_1(1) + finalPoint.t_v(1) + finalPoint.t_2(1), finalPoint.t_1(1) + finalPoint.t_v(1) + finalPoint.t_2(1)]
    i = i + 1;
    for q_int = 1:cfg.dim
        if t < finalPoint.t_1(q_int)
            q_1 = node1(q_int) + node1(q_int+2) * t + 1/2 * finalPoint.a_1(q_int) * t^2;
        elseif t < finalPoint.t_1(q_int) + finalPoint.t_v(q_int)
            q_1 = q_1_end(q_int) + v_1_end(q_int) * (t - finalPoint.t_1(q_int));
        else
            q_1 = q_v_end(q_int) + v_v_end(q_int) * (t - finalPoint.t_v(q_int) - finalPoint.t_1(q_int)) + ...
                1/2 * finalPoint.a_2(q_int) * (t - finalPoint.t_v(q_int) - finalPoint.t_1(q_int))^2;
        end
        q_list_vis(q_int,i) = q_1;
    end
end
end
