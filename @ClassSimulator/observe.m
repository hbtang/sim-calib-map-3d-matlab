function observe( this )
%OBSERVE renew current observation and save into this.mk

this.mk = struct('lp',[],'id',[],'rvec',[],'tvec',[]);
lp = this.lp;

numMkId = numel(this.map.mks.id);
ps2d_w_b = this.ps2d_w_b;
T3d_w_b = FunPs2d2T3d(ps2d_w_b);
T3d_b_c = this.calib.T3d_b_c;
T3d_c_w = inv(T3d_w_b*T3d_b_c);

angleOfViewH = this.config.angleOfViewH;
angleOfViewV = this.config.angleOfViewV;
distRangeMin = this.config.distRangeMin;
distRangeMax = this.config.distRangeMax;

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
            depth_c_m >= distRangeMin && depth_c_m <= distRangeMax
        this.mk.lp = [this.mk.lp; lp];
        this.mk.id = [this.mk.id; mkId];
        this.mk.rvec = [this.mk.rvec; rvec_c_m.'];
        this.mk.tvec = [this.mk.tvec; tvec_c_m.'];
    end    
end

end

