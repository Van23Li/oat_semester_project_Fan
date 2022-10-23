function [closestNode, closestNode_int, M, I, T] = nearest_kd(sample, RRTree, cfg)
RRTree = [RRTree, 1.5*RRTree];
v_1 = RRTree(cfg.dim+1 : 2*cfg.dim,:);
v_2 = sample(cfg.dim+1 : 2*cfg.dim);
p_1 = RRTree(1 : cfg.dim,:);
p_2 = sample(1 : cfg.dim);

% judge if a1 is positive or negative
p_acc = (v_1 + v_2) .* (abs(v_2 - v_1)) ./ cfg.a_max' / 2;
sigma = sign(p_2 - p_1 - p_acc);
a_1 =  sigma .* cfg.a_max';
a_2 = -sigma .* cfg.a_max';
v_limit = sigma .* cfg.v_max'

%% calculate minmum time
a = a_1;
b = 2 * v_1;
c = (v_2.^2 - v_1.^2) ./ (2 * a_2) - (p_2 - p_1);
q = -1/2 * (b + sign(b) .* sqrt(b.^2 - 4 * a .* c));
t_1 = (sign(a) ~= sign(b)) .* q./a + (sign(a) == sign(b)) .* c./q;
t_2 = (v_2 - v_1) ./ a_2 + t_1;

% check whether the solution satisfies the velocity limits
velocity_valided = v_1 + a_1 .* t_1 <= cfg.v_max' & v_1 + a_1 .* t_1 >= cfg.v_min';
t_1 = velocity_valided .* t_1 + ~velocity_valided .* (v_limit - v_1) ./ a_1;
t_2 = velocity_valided .* t_2 + ~velocity_valided .* (v_2 - v_limit) ./ a_2;

t = t_1 + t_2;
t_min = min(t,[],1);

%% calculate infeasible time interval
a_1 = -sigma .* cfg.a_max';
a_2 =  sigma .* cfg.a_max';
v_limit = -sigma .* cfg.v_max'

a = a_1;
b = 2 * v_1;
c = (v_2.^2 - v_1.^2) ./ (2 * a_2) - (p_2 - p_1);
% Only calculate when fall in region 1
if sum(b.^2 - 4 * a .* c >= 0)
    q = -1/2 * (b + sign(b) .* sqrt(b.^2 - 4 * a .* c));
    t_1_in_max = (sign(a) ~= sign(b)) .* q./a + (sign(a) == sign(b)) .* c./q;
    t_1_in_min = (sign(a) == sign(b)) .* q./a + (sign(a) ~= sign(b)) .* c./q;
    t_2 = (v_2 - v_1) ./ a_2 + t_1;

    % check whether the solution satisfies the velocity limits
    velocity_valided = v_1 + a_1 .* t_1_in_max <= cfg.v_max' & v_1 + a_1 .* t_1_in_max >= cfg.v_min';
    t_1 = velocity_valided .* t_1 + ~velocity_valided .* (v_limit - v_1) ./ a_1;
    t_2 = velocity_valided .* t_2 + ~velocity_valided .* (v_2 - v_limit) ./ a_2;

    t = t_1 + t_2;
end

%% Find the closest node.
[M,I] = min(t_min);
closestNode = RRTree(1:2*cfg.dim, I);
closestNode_int = RRTree(end, I);
T = t(:,I);
end