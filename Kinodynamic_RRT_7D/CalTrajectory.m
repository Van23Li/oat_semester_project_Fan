function Result = CalTrajectory(x1, x2, cfg)
    T = Dist_kd(x1, x2, cfg);
    Result = Steer(x1, x2, T, cfg); %[a_1, v_limit, a_2, t_1, t_v, t_2]
end