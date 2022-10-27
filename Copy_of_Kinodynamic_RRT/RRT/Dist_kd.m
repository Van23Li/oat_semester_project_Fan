function out = Dist_kd (x1, x2, cfg)
if sum(x1 == x2(1:2*cfg.dim,1)) == length(x1)
    out = 0;
    return
end
v_1 = x2(cfg.dim+1 : 2*cfg.dim,:);
v_2 = x1(cfg.dim+1 : 2*cfg.dim);
p_1 = x2(1 : cfg.dim,:);
p_2 = x1(1 : cfg.dim);

% judge if a1 is positive or negative
p_acc = 1/2 * (v_1 + v_2) .* (abs(v_2 - v_1)) ./ cfg.a_max';
sigma = sign(p_2 - p_1 - p_acc);
a_1 =  sigma .* cfg.a_max';
a_2 = -sigma .* cfg.a_max';
v_limit = sigma .* cfg.v_max';

%% calculate minmum time
a = a_1;
b = 2 * v_1;
c = (v_2.^2 - v_1.^2) ./ (2 * a_2) - (p_2 - p_1);
q = -1/2 * (b + sign(b) .* sqrt(b.^2 - 4 * a .* c));
t_1 = (sign(a) ~= sign(b)) .* q./a + (sign(a) == sign(b)) .* c./q;
t_1(sign(b)==0) = sign(a(sign(b)==0)) .* sqrt(-4 * a(sign(b)==0) .* c(sign(b)==0)) ./ (2*a(sign(b)==0));
t_2 = (v_2 - v_1) ./ a_2 + t_1;

% check whether the solution satisfies the velocity limits
velocity_valided = v_1 + a_1 .* t_1 <= cfg.v_max' & v_1 + a_1 .* t_1 >= cfg.v_min';
t_1 = velocity_valided .* t_1 + ~velocity_valided .* (v_limit - v_1) ./ a_1;
t_v = ~velocity_valided .* ((v_1.^2 + v_2.^2 - 2*v_limit.^2)./(2*v_limit .* a_1) + (p_2 - p_1)./v_limit);
t_2 = velocity_valided .* t_2 + ~velocity_valided .* (v_2 - v_limit) ./ a_2;

t = t_1 + t_2 + t_v;

%% calculate infeasible time interval
a_1 = -sigma .* cfg.a_max';
a_2 =  sigma .* cfg.a_max';
v_limit = -sigma .* cfg.v_max';

a = a_1;
b = 2 * v_1;
c = (v_2.^2 - v_1.^2) ./ (2 * a_2) - (p_2 - p_1);
% Only calculate when fall in region 1
% if sum(b.^2 - 4 * a .* c >= 0)
%     q = -1/2 * (b + sign(b) .* sqrt(b.^2 - 4 * a .* c));
%     t_1_in_max = (sign(a) ~= sign(b)) .* q./a + (sign(a) == sign(b)) .* c./q;
%     t_1_in_min = (sign(a) == sign(b)) .* q./a + (sign(a) ~= sign(b)) .* c./q;
%     t_2 = (v_2 - v_1) ./ a_2 + t_1;
% 
%     % check whether the solution satisfies the velocity limits
%     velocity_valided = v_1 + a_1 .* t_1_in_max <= cfg.v_max' & v_1 + a_1 .* t_1_in_max >= cfg.v_min';
%     t_1 = velocity_valided .* t_1 + ~velocity_valided .* (v_limit - v_1) ./ a_1;
%     t_2 = velocity_valided .* t_2 + ~velocity_valided .* (v_2 - v_limit) ./ a_2;
% 
%     t = t_1 + t_2;
% end
%%
out = max(t,[],1);
end