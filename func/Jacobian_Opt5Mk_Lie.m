function [ mat_jacobian, mat_jacobian_bc, mat_jacobian_wb, mat_jacobian_wm ] = Jacobian_Opt5Mk_Lie( ...
    rvec_b_c, tvec_b_c, rvec_w_b, tvec_w_b, rvec_w_m, tvec_w_m,...
    tvec_m_f, mat_camera, vec_distortion)
%JACOBIAN_OPT5MK compute the jacobian of each mark constraint for opt5
% return jacobian of undistored image cordinates

[ T3d_b_c, R3d_b_c ] = FunVec2Trans3d( rvec_b_c, tvec_b_c );
[ T3d_w_b, R3d_w_b ] = FunVec2Trans3d( rvec_w_b, tvec_w_b );
[ T3d_w_m, R3d_w_m ] = FunVec2Trans3d( rvec_w_m, tvec_w_m );
T3d_c_m = inv(T3d_w_b*T3d_b_c)*T3d_w_m;
[ rvec_c_m, tvec_c_m ] = FunTrans2Vec3d( T3d_c_m );
tvecbar_c_f = T3d_c_m*[tvec_m_f;1];
tvec_c_f = tvecbar_c_f(1:3);

% J0: jacobian of undistorted image cordinate [u0,v0], to tvec_c_pt
% todo ...
x_c_f = tvec_c_f(1);
y_c_f = tvec_c_f(2);
z_c_f = tvec_c_f(3);
J0_temp = [...
    1/z_c_f 0 -x_c_f/(z_c_f^2);...
    0 1/z_c_f -y_c_f/(z_c_f^2);...
    0 0 0;
    ];
J0bar = mat_camera*J0_temp;
J0 = J0bar(1:2,:);

% J1: jacobian of tvec_c_pt to [rvec_b_c; tvec_b_c]
tvec_temp_1 = R3d_w_b.'*(R3d_w_m*tvec_m_f+tvec_w_m-tvec_w_b) - tvec_b_c;
J1_r = -DiffRotVec2Lie( rodrigues(-rvec_b_c), tvec_temp_1 );
J1_t = -R3d_b_c.';
J1 = [J1_r J1_t];

% J2: jacobian of tvec_c_pt to [rvec_w_b; tvec_w_b]
tvec_temp_2 = R3d_w_m*tvec_m_f+tvec_w_m-tvec_w_b;
J2_r = R3d_b_c.'*(-DiffRotVec2Lie(rodrigues(-rvec_w_b), tvec_temp_2));
J2_t = -R3d_b_c.'*R3d_w_b.';
J2 = [J2_r J2_t];

% J3: jacobian of tvec_c_pt to [rvec_w_m; tvec_w_m]
J3_r = R3d_b_c.'*R3d_w_b.'*DiffRotVec2Lie(rodrigues(rvec_w_m), tvec_m_f);
J3_t = R3d_b_c.'*R3d_w_b.';
J3 = [J3_r J3_t];

% combine
mat_jacobian = J0*[J1 J2 J3];
mat_jacobian_bc = J0*[J1_r J1_t(:,1:2)];
mat_jacobian_wb = J0*[J2_t(:,1:2), J2_r(:,3)];
mat_jacobian_wm = J0*J3;

end

