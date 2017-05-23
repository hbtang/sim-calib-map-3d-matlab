function tvec_c_m = FunTveccm( this, rvec_b_c, tvec_b_c, se2_w_b, tvec_w_m )
%FUNTVECWM 此处显示有关此函数的摘要
%   此处显示详细说明

T_b_c = FunVec2Trans3d( rvec_b_c, tvec_b_c );
T_w_b = FunPs2d2T3d(se2_w_b);
T_w_c = T_w_b*T_b_c;
R_w_c = T_w_c(1:3,1:3);
t_w_c = T_w_c(1:3,4);

T_c_w = eye(4);
T_c_w(1:3,1:3) = R_w_c.';
T_c_w(1:3,4) = - R_w_c.'*t_w_c;

tvec_c_m_bar = T_c_w * [tvec_w_m;1];
tvec_c_m = tvec_c_m_bar(1:3);

end

