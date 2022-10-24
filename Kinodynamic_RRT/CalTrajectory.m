function [a_1, v_limit, a_2, t_1, t_v, t_2] = CalTrajectory(x1, x2, cfg)
    T = Dist_kd(x1, x2, cfg);
    [~, a_1, v_limit, a_2, t_1, t_v, t_2] = Steer(x1, x2, T, cfg);
end