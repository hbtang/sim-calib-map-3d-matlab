function RefreshByTbcgc( this )
%REFRESHBYTBCGC refresh calibration properties according to _b_cg and _cg_c

%% refresh vec_b_cg and vec_cg_c
T3d_b_cg = this.T3d_b_cg;
T3d_cg_c = this.T3d_cg_c;
[rvec_b_cg, tvec_b_cg] = FunTrans2Vec3d(T3d_b_cg);
[rvec_cg_c, tvec_cg_c] = FunTrans2Vec3d(T3d_cg_c);
this.rvec_b_cg = rvec_b_cg;
this.tvec_b_cg = tvec_b_cg;
this.rvec_cg_c = rvec_cg_c;
this.tvec_cg_c = tvec_cg_c;

%% refresh pvec_g_c and dist_g_c
[this.pvec_g_c, this.dist_g_c] = FunRot2Grnd(rvec_cg_c, tvec_cg_c);

%% refresh _b_c
T3d_b_c = T3d_b_cg*T3d_cg_c;
[rvec_b_c, tvec_b_c] = FunTrans2Vec3d(T3d_b_c);
this.T3d_b_c = T3d_b_c;
this.rvec_b_c = rvec_b_c;
this.tvec_b_c = tvec_b_c;

end

