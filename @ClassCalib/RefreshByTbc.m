function RefreshByTbc( this )
%REFRESHBYTBC refresh class calib according to Tbc

T3d_b_c = this.T3d_b_c;
rvec_b_c = rodrigues(T3d_b_c(1:3, 1:3));
tvec_b_c = T3d_b_c(1:3, 4);
this.rvec_b_c = rvec_b_c;
this.tvec_b_c = tvec_b_c;

%% obtain T_b_cg
vec_cz_b = T3d_b_c(1:3,1:3)*[0;0;1];
vec_cgx_b = [vec_cz_b(1:2);0]/norm([vec_cz_b(1:2);0]);
vec_cgy_b = cross([0;0;1],vec_cgx_b);
R3d_b_cg = [vec_cgx_b, vec_cgy_b, [0;0;1]];
rvec_b_cg = rodrigues(R3d_b_cg);
tvec_b_cg = [this.tvec_b_c(1:2);0];
T3d_b_cg = FunVec2Trans3d(rvec_b_cg, tvec_b_cg);
this.T3d_b_cg = T3d_b_cg;
this.rvec_b_cg = rvec_b_cg;
this.tvec_b_cg = tvec_b_cg;

%% obtain T_cg_c
T3d_cg_c = inv(T3d_b_cg)*T3d_b_c;
[rvec_cg_c, tvec_cg_c] = FunTrans2Vec3d(T3d_cg_c);
[pvec_g_c, dist_g_c] = FunRot2Grnd(rvec_cg_c, tvec_cg_c);

this.T3d_cg_c = T3d_cg_c;
this.rvec_cg_c = rvec_cg_c;
this.tvec_cg_c = tvec_cg_c;
this.pvec_g_c = pvec_g_c;
this.dist_g_c = dist_g_c;

end

