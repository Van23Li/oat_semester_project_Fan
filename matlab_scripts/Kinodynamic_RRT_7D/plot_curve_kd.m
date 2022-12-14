function plot_curve_kd(node1, node2, cfg, handle, color, linewidth)
if nargin < 5
    color = [0 0 1];
    linewidth = 0.5;
end

Time = Dist_kd(node2, node1, cfg);
finalPoint = Steer(node1, node2, Time, cfg);

q_1_end = node1(1:cfg.dim) + node1(cfg.dim + 1:2 * cfg.dim) .* finalPoint.t_1 + 1/2 .* finalPoint.a_1 .* finalPoint.t_1.^2;
v_1_end = node1(cfg.dim + 1:2 * cfg.dim) + finalPoint.a_1 .* finalPoint.t_1;
q_v_end = q_1_end + v_1_end .* finalPoint.t_v;
v_v_end = v_1_end;
q_2_end = q_v_end + v_v_end .* finalPoint.t_2 + 1/2 * finalPoint.a_2 .* finalPoint.t_2.^2;
v_2_end = v_v_end + finalPoint.a_2 .* finalPoint.t_2;

q_list_vis = [];
i = 0;
for t = linspace(0, finalPoint.t_1(1) + finalPoint.t_v(1) + finalPoint.t_2(1),100)
    i = i + 1;
    for j = 1:cfg.dim
        if t < finalPoint.t_1(j)
            q_1 = node1(j) + node1(j+2) * t + 1/2 * finalPoint.a_1(j) * t^2;
        elseif t < finalPoint.t_1(j) + finalPoint.t_v(j)
            q_1 = q_1_end(j) + v_1_end(j) * (t - finalPoint.t_1(j));
        else
            q_1 = q_v_end(j) + v_v_end(j) * (t - finalPoint.t_v(j) - finalPoint.t_1(j)) + ...
                1/2 * finalPoint.a_2(j) * (t - finalPoint.t_v(j) - finalPoint.t_1(j))^2;
        end
        q_list_vis(j,i) = q_1;
    end
end

if cfg.display2 && cfg.dim == 2
    figure(handle.fig_handle2)
    hold on
    plot(handle.ax_h2, q_list_vis(2,:), q_list_vis(1,:),'LineStyle','-','Color',color,'LineWidth',linewidth);
    %         plot(handle.ax_h2, q_list_vis(2,end), q_list_vis(1,end),'bo');
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
    plot(handle.ax_h3, tmp_list(1,:), tmp_list(2,:),'LineStyle','-','Color',color,'LineWidth',linewidth);
    hold off
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