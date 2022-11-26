function obs = generate_obs(max_num, min_num, max_radius, min_radius, cfg)
num = round(min_num + (max_num - min_num) * rand);
obs.centers = [];
obs.radiuses = [];
i = 0;
while(i < num)
    centers = [cfg.q_min(end - 1) + (8 - cfg.q_min(end - 1)) * rand, ...
        cfg.q_min(end) + (8 - cfg.q_min(end)) * rand];
    radius = round(min_radius + (max_radius - min_radius) * rand);
    
    if norm(centers) - radius > 4
        obs.centers = [obs.centers; centers];
        obs.radiuses = [obs.radiuses; radius];
        i = i + 1;
    end 
end
end