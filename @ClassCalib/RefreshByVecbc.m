function RefreshByVecbc( this )
%REFRECHBYVECBC refresh class calib according to rvec_b_c and tvec_b_c
this.T3d_b_c = FunVec2Trans3d(this.rvec_b_c, this.tvec_b_c);
RefreshByTbc( this );
end

