function InitMap( this, measure, calib, flag )
%INITMAP init map from measurement

if nargin < 4
    flag.bKfsOdo = true;
    flag.bMksLin = true;
    flag.bMksRot = true;
end

odo = measure.odo;
mk = measure.mk;
disp('Initializing map...');
kfs = this.kfs;
mks = this.mks;

%% init keyframes from odometry
if flag.bKfsOdo
kfs = struct('lp',[],'rvec_w_b',[],'tvec_w_b',[],...
    'ps2d_w_b',[],'numKfs',[]);
for i = 1:odo.num
    ps2d_w_b = [odo.x(i); odo.y(i); odo.theta(i)];
    rvec_w_b = [0;0;odo.theta(i)];
    tvec_w_b = [odo.x(i); odo.y(i); 0];
    kfs.lp(i,1) = odo.lp(i);
    kfs.ps2d_w_b(i,:) = ps2d_w_b.';
    kfs.rvec_w_b(i,:) = rvec_w_b.';
    kfs.tvec_w_b(i,:) = tvec_w_b.';
end
kfs.numKfs = odo.num;
this.kfs = kfs;
end

%% init marks from mark observation
T3d_b_c = calib.T3d_b_c;
for i = 1:mk.numMkId
    mkId = mk.vecMkId(i);
    tmp = find(mk.id == mkId, 1);
    lp = mk.lp(tmp);
    rvec_c_m = mk.rvec(tmp,:).';
    tvec_c_m = mk.tvec(tmp,:).';
    T3d_c_m = FunVec2Trans3d(rvec_c_m, tvec_c_m);
    
    tmp = find(kfs.lp == lp, 1);
    rvec_w_b = kfs.rvec_w_b(tmp,:).';
    tvec_w_b = kfs.tvec_w_b(tmp,:).';
    T3d_w_b = FunVec2Trans3d(rvec_w_b, tvec_w_b);
    
    T3d_w_m = T3d_w_b*T3d_b_c*T3d_c_m;
    [rvec_w_m, tvec_w_m] = FunTrans2Vec3d(T3d_w_m);
    
    mks.id(i,1) = mkId;
    if flag.bMksLin    
        mks.tvec_w_m(i,:) = tvec_w_m;  
    end
    if flag.bMksRot
        mks.rvec_w_m(i,:) = rvec_w_m;
    end
end
mks.numMks = mk.numMkId;
this.mks = mks;

disp('Map initialized.');
disp(' ');

end

