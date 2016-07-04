function PruneData( this, threshDistOdo, threshAngleOdo )
%PRUNEDATA prune raw input data

%% init

if nargin == 1
    threshDistOdo = 500;
    threshAngleOdo = 5*pi/180;
end


%% prune odometry data
xRef = this.odo.x(1);
yRef = this.odo.y(1);
thetaRef = this.odo.theta(1);
lpRef = this.odo.lp(1);
odo_new = struct('lp',lpRef, 'x',xRef,'y',yRef,'theta',thetaRef);

for i = 2:this.odo.num
    xNow = this.odo.x(i);
    yNow = this.odo.y(i);
    thetaNow = this.odo.theta(i);
    lpNow = this.odo.lp(i);
    
    dx = xNow - xRef;
    dy = yNow - yRef;
    dl = norm([dx;dy]);
    dtheta = FunPrdCnst(thetaNow - thetaRef, pi, -pi);
    
    if (dl > threshDistOdo) || (abs(dtheta) > threshAngleOdo)
        odo_new.lp = [odo_new.lp; lpNow];
        odo_new.x = [odo_new.x; xNow];
        odo_new.y = [odo_new.y; yNow];
        odo_new.theta = [odo_new.theta; thetaNow];
        
        lpRef = lpNow;
        xRef = xNow;
        yRef = yNow;
        thetaRef = thetaNow;
    end
end
odo_new.num = numel(odo_new.lp);
this.odo = odo_new;
disp(['Odometry info prunning is done, ', num2str(odo_new.num), ' records remained.']);

%% prune mark data
mk_new = struct('lp', [], 'id', [], ...
    'rvec', [], 'tvec', [], 'num', [], ...
    'numMkId', [], 'vecMkId', []);
for i = 1:this.mk.num
    tmp = find(odo_new.lp == this.mk.lp(i), 1);
    if numel(tmp) ~= 0
        mk_new.lp = [mk_new.lp; this.mk.lp(i)];
        mk_new.id = [mk_new.id; this.mk.id(i)];
        mk_new.rvec = [mk_new.rvec; this.mk.rvec(i,:)];
        mk_new.tvec = [mk_new.tvec; this.mk.tvec(i,:)];
    end
end
mk_new.num = numel(mk_new.lp);
mk_new.vecMkId = unique(mk_new.id);
mk_new.numMkId = numel(mk_new.vecMkId);
this.mk = mk_new;
disp(['Marker observation info prunning is done, ', num2str(mk_new.num), ' records remained.'])
disp(' ');

end

