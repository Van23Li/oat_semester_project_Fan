% ��⵱ǰ����㵽�½��ķ����ϣ����е��ڵ�ͼ�ڲ�
function feasible = checkPath(n,newPos,obstacle,dim)
feasible = true;
for r=0: 0.1: norm(n-newPos, 2)
    dir = n + (newPos - n)/norm(n-newPos, 2) * r;
    dir = mod(dir,360);
%     dir_border = [dir < 0,dir > 360];
%     if sum(sum(dir_border)) > 0
%         dir = mod(dir,360);
%     end
    fv = ThreeLinkRobot (dir,dim);
    if CollisionCheck(fv, obstacle)
        feasible = false;
        break;
    end
end
