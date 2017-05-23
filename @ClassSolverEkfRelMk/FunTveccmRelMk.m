function tvec_c_m = FunTveccmRelMk(this, rvec_b_c, tvec_b_c,...
            se2_w_b, se2_w_bkf, tvec_ckf_m)
%FUNTVECCMRELMK 

T_b_c = FunVec2Trans3d( rvec_b_c, tvec_b_c );
T_w_b = FunPs2d2T3d(se2_w_b);
T_w_bkf = FunPs2d2T3d(se2_w_bkf);

T_ckf_c = (T_w_bkf*T_b_c) \ T_w_b * T_b_c;

tvec_c_m_bar = T_ckf_c \ [tvec_ckf_m;1];
tvec_c_m = tvec_c_m_bar(1:3);

end

