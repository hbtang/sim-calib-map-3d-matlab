function Observe( this )
%OBSERVE renew current observation and save into this.mk

this.mk = struct('lp',[],'id',[],'rvec',[],'tvec',[],'image',[]);
lp = this.lp;

numMkId = numel(this.map.mks.id);
ps2d_w_b = this.ps2d_w_b;
T3d_w_b = FunPs2d2T3d(ps2d_w_b);
T3d_b_c = this.calib.T3d_b_c;
T3d_c_w = inv(T3d_w_b*T3d_b_c);

angleOfViewH = this.setting.range.angleofview_h;
angleOfViewV = this.setting.range.angleofview_v;
depthMin = this.setting.range.depth_min;
depthMax = this.setting.range.depth_max;

%% select mark by sensor range
for i = 1:numMkId
    mkId = this.map.mks.id(i);
    tvec_w_m = this.map.mks.tvec_w_m(i,:).';
    rvec_w_m = this.map.mks.rvec_w_m(i,:).';
    T3d_w_m = FunVec2Trans3d(rvec_w_m, tvec_w_m);
    T3d_c_m = T3d_c_w*T3d_w_m;
    [rvec_c_m, tvec_c_m] = FunTrans2Vec3d(T3d_c_m);
    
    depth_c_m = tvec_c_m(3);
    angleH_c_m = atan2(tvec_c_m(1),tvec_c_m(3));
    angleV_c_m = atan2(tvec_c_m(2),tvec_c_m(3));
    
    if abs(angleH_c_m) < angleOfViewH/2 && ...
            abs(angleV_c_m) < angleOfViewV/2 && ...
            depth_c_m >= depthMin && depth_c_m <= depthMax
        this.mk.lp = [this.mk.lp; lp];
        this.mk.id = [this.mk.id; mkId];
        this.mk.rvec = [this.mk.rvec; rvec_c_m.'];
        this.mk.tvec = [this.mk.tvec; tvec_c_m.'];
    end
end

%% prune by image
numMk = numel(this.mk.lp);
mkImgGood = struct('lp',[],'id',[],'rvec',[],'tvec',[],'image',[]);
mattvec_m_f = this.setting.aruco.mattvec;
tvec_m_f1 = mattvec_m_f(1,:).';
tvec_m_f2 = mattvec_m_f(2,:).';
tvec_m_f3 = mattvec_m_f(3,:).';
tvec_m_f4 = mattvec_m_f(4,:).';

mat_camera = this.calib.mat_camera;
vec_distortion = this.calib.vec_distortion;
thresh_image = 20;

for i = 1:numMk
    lp = this.mk.lp(i);
    id = this.mk.id(i);
    rvec_c_m = this.mk.rvec(i,:).';
    tvec_c_m = this.mk.tvec(i,:).';
    
    image_f1 = ProjXYZ2UV(rvec_c_m, tvec_c_m, tvec_m_f1, mat_camera, vec_distortion);
    image_f2 = ProjXYZ2UV(rvec_c_m, tvec_c_m, tvec_m_f2, mat_camera, vec_distortion);
    image_f3 = ProjXYZ2UV(rvec_c_m, tvec_c_m, tvec_m_f3, mat_camera, vec_distortion);
    image_f4 = ProjXYZ2UV(rvec_c_m, tvec_c_m, tvec_m_f4, mat_camera, vec_distortion);
    image_f = [image_f1; image_f2; image_f3; image_f4];
    image_u = [image_f1(1); image_f2(1); image_f3(1); image_f4(1)];
    image_v = [image_f1(2); image_f2(2); image_f3(2); image_f4(2)];
    
    %% continue if out of image bound
    if max(image_u) > this.calib.image_width || min(image_u) < 0 ...
            || max(image_v) > this.calib.image_height || min(image_v) < 0
        continue;
    end
    
    %% compute approximate area of mark in image
    dimage_13 = image_f1 - image_f3;
    dimage_24 = image_f2 - image_f4;
    approxarea = abs(dimage_13(1)*dimage_24(2) - dimage_13(2)-dimage_24(1));
    if approxarea > thresh_image
        mkImgGood.lp = [mkImgGood.lp; lp];
        mkImgGood.id = [mkImgGood.id; id];
        mkImgGood.rvec = [mkImgGood.rvec; rvec_c_m.'];
        mkImgGood.tvec = [mkImgGood.tvec; tvec_c_m.'];
        mkImgGood.image = [mkImgGood.image; image_f.'];
    end
end
this.mk = mkImgGood;

end

