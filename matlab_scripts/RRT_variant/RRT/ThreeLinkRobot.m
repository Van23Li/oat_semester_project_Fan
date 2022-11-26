function fv = ThreeLinkRobot (cspace, dimension)
% Simulate a simple six link revolute chain

if dimension == 3
    l = [2; 3; 5];
    h = [0.5; 0.5; 0.5];
elseif dimension == 2
    l = [4; 4];
    h = [0.5; 0.5];
end
link = [];

for i = 1:dimension
    link = [link; boxFV(0,l(i),-h(i),h(i))];
end

fv = link(1);

for i = 1:dimension - 1
    fv = appendFV(link(i+1), transformFV(fv, cspace(i), [l(i+1) 0]));
end

fv = transformFV(fv, cspace(end), [0 0]);
end