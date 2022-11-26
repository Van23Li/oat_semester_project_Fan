function x = RandomSampleThreeLink (obstacle,dim)
% Generate a random freespace configuration for the robot

while true
    x = 360*rand(dim,1);
    
    fv = ThreeLinkRobot (x,dim);
    
    if (~CollisionCheck(fv, obstacle))
        return
    end
end