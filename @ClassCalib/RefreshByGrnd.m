function RefreshByGrnd( this )
%REFRESHBYGRND refresh calibration properties according to pvec_g_c and dist_g_c

[this.rvec_cg_c, this.tvec_cg_c] = FunGrnd2Rot(this.pvec_g_c, this.dist_g_c);
RefreshByVecbcgc(this);

end

