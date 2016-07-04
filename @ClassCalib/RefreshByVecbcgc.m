function RefreshByVecbcgc( this )
%REFRESHBYVECBCGC refresh calibration properties according to _b_cg and _cg_c

this.T3d_b_cg = FunVec2Trans3d(this.rvec_b_cg, this.tvec_b_cg);
this.T3d_cg_c = FunVec2Trans3d(this.rvec_cg_c, this.tvec_cg_c);
RefreshByTbcgc(this);

end

