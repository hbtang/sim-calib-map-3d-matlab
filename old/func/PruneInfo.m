function [ mark_new, odo_new ] = PruneInfo( mark, odo )
%PRUNEINFO: prune raw info data, by theshold

%% init
threshDistMk = 50;
threshAngleMk = 3*pi/180;
threshDistOdo = 100;
threshAngleOdo = 5*pi/180;

%% prune mark info
mkIdLast = -1;
mark_new.size = mark.size;
mark_new.stamp = []; mark_new.id = []; 
mark_new.rvec = []; mark_new.tvec = [];
for i = 1:mark.num
    % if new mark detected, choose
    if mkIdLast ~= mark.id(i)
        mkIdLast = mark.id(i);
        mark_new.stamp(end+1,1) = mark.stamp(i);
        mark_new.id(end+1,1) = mark.id(i);
        mark_new.rvec(end+1,:) = mark.rvec(i,:);
        mark_new.tvec(end+1,:) = mark.tvec(i,:);
        continue;
    end
    
    % if threshold satisfied
    stampLast = mark_new.stamp(end);
    stampNow = mark.stamp(i);
    
    dx = odo.x(stampNow) - odo.x(stampLast);
    dy = odo.y(stampNow) - odo.y(stampLast);
    dl = norm([dx;dy]);
    dtheta = FunPrdCnst(odo.theta(stampNow) - odo.theta(stampLast), -pi, pi);
    
    if (abs(dl) > threshDistMk) || (abs(dtheta) > threshAngleMk)
        mark_new.stamp(end+1,1) = mark.stamp(i);
        mark_new.id(end+1,1) = mark.id(i);
        mark_new.rvec(end+1,:) = mark.rvec(i,:);
        mark_new.tvec(end+1,:) = mark.tvec(i,:);
        continue;
    end    
end
mark_new.num = numel(mark_new.stamp);

%% prune odo info
mkIdTmp = 1;
odo_new.stamp = odo.stamp(1);
odo_new.x = odo.x(1);
odo_new.y = odo.y(1);
odo_new.theta = odo.theta(1);

for i = 2:odo.num
    dx = odo_new.x(end) - odo.x(i);
    dy = odo_new.y(end) - odo.y(i);
    dtheta = odo_new.theta(end) - odo.theta(i);
    dl = sqrt(dx*dx + dy*dy);
    
    % if same stamp in mark_new, choose
    if (mark_new.stamp(mkIdTmp) == i)
        odo_new.stamp(end+1,1) = odo.stamp(i);
        odo_new.x(end+1,1) = odo.x(i);
        odo_new.y(end+1,1) = odo.y(i);
        odo_new.theta(end+1,1) = odo.theta(i);
        if mkIdTmp < mark_new.num
            mkIdTmp = mkIdTmp + 1;
        end
        
    % if threshold satisfied    
    elseif (dl > threshDistOdo) || (abs(dtheta) > threshAngleOdo)
        odo_new.stamp(end+1,1) = odo.stamp(i);
        odo_new.x(end+1,1) = odo.x(i);
        odo_new.y(end+1,1) = odo.y(i);
        odo_new.theta(end+1,1) = odo.theta(i);  
    end
end
odo_new.num = numel(odo_new.stamp);

%% refresh odo_new.stamp and mark_new.stamp
[~,mark_new.stamp,~] = intersect(odo_new.stamp,mark_new.stamp);
odo_new.stamp = (1:odo_new.num).';

end

