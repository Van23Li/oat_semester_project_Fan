function x = RandomSampleThreeLink (obstacle,q_min,q_max,dim)
% Generate a random freespace configuration for the robot
% return [q2; q1]
while true
%     x = 360*rand(dim,1);
    x = q_min(1:dim)' + (q_max(1:dim) - q_min(1:dim))' .* rand(dim,1);
    fv = ThreeLinkRobot (x,dim);
    
    if (~CollisionCheck(fv, obstacle))
        return
    end
end