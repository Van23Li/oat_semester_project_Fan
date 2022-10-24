function [newPoint, a_1, v_limit, a_2, t_1, t_v, t_2] = Steer(closestNode, sample, T, cfg)
newPoint = sample;
v_1 = closestNode(cfg.dim+1 : 2*cfg.dim);
v_2 = sample(cfg.dim+1 : 2*cfg.dim);
p_1 = closestNode(1 : cfg.dim);
p_2 = sample(1 : cfg.dim);

% calculate the acceleration
a = T.^2;
b = 2*T .* (v_1 + v_2) - 4*(p_2 - p_1);
c = -(v_2 - v_1).^2;
q = -1/2 * (b + sign(b) .* sqrt(b.^2 - 4 * a .* c));
% a_1 = (sign(b) ~= 0) .* (sign(a) ~= sign(b)) .* q./a + ...
%     (sign(b) ~= 0) .* (sign(a) == sign(b)) .* c./q + ...
%     ~sign(b) .* sqrt(-4 * a .* c) ./ (2*a);
a_1 = q./a;
a_2 = -a_1;

t_1 = 1/2 * ((v_2 - v_1)./a_1 + T);
t_v = 0 * t_1;
t_2 = T - t_1;
v_limit = 0 * t_1;

% check whether the solution satisfies the velocity limits
velocity_valided = v_1 + a_1 .* t_1 <= cfg.v_max' & v_1 + a_1 .* t_1 >= cfg.v_min';
if sum(velocity_valided) < length(velocity_valided)
    v_limit = sign(a_1) .* cfg.v_max';
    a_1 = velocity_valided .* a_1 + ...
        ~velocity_valided .* ((v_limit - v_1).^2 + (v_limit - v_2).^2)./(2*(v_limit .* T - (p_2 - p_1)));
    a_2 = -a_1;
    t_1 = velocity_valided .* t_1 + ~velocity_valided .* (v_limit - v_1) ./ a_1;
    t_v = velocity_valided .* t_v + ~velocity_valided .* ((v_1.^2 + v_2.^2 - 2*v_limit.^2)./(2*v_limit .* a_1) + (p_2 - p_1)./v_limit);
    t_2 = velocity_valided .* t_2 + ~velocity_valided .* (v_2 - v_limit) ./ a_2;
end
    if sum(t_1) < 0 || sum(t_2) < 0
        a=2
    end
end