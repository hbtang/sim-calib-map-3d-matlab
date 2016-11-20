function pt_img_ret = ProjXYZ2UV( rvec_c_m, tvec_c_m, tvec_m_pt, mat_camera, vec_distortion )
%PROJXYZ2UV Project 3D space point into 2D image

k1 = vec_distortion(1);
k2 = vec_distortion(2);
p1 = vec_distortion(3);
p2 = vec_distortion(4);
k3 = vec_distortion(5);

R_c_m = rodrigues(rvec_c_m);
T_c_m = [R_c_m tvec_c_m; 0 0 0 1];
tvecbar_m_p = [tvec_m_pt; 1];
tvecbar_c_p = T_c_m*tvecbar_m_p;
tvec_c_p = tvecbar_c_p(1:3,1);

tvec_c_p_normal = tvec_c_p/tvec_c_p(3);
x_c_p_normal = tvec_c_p_normal(1);
y_c_p_normal = tvec_c_p_normal(2);
r = norm(tvec_c_p_normal(1:2));

x_c_p_undist = x_c_p_normal*(1+k1*r^2+k2*r^4+k3*r^6) ...
    + 2*p1*x_c_p_normal*y_c_p_normal + p2*(r^2+2*x_c_p_normal^2);
y_c_p_undist = y_c_p_normal*(1+k1*r^2+k2*r^4+k3*r^6) ...
    + 2*p2*x_c_p_normal*y_c_p_normal + p1*(r^2+2*y_c_p_normal^2);
tvec_c_p_undist = [x_c_p_undist; y_c_p_undist; 1];

ptbar_img = mat_camera*tvec_c_p_undist;
pt_img_ret = ptbar_img(1:2);

end

