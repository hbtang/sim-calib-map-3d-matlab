function [ rvec_cg_c, tvec_cg_c ] = FunGrnd2Rot( pvec_g_c, dist_g_c )
%FUNGRND2ROT calculate rotation matrix from camera frame to its projection
%on ghe ground, according to ground plane w.r.t. camera frame

z_cg_c = pvec_g_c/norm(pvec_g_c);
% camera y axis pointed to ground
if z_cg_c(2) > 0
    z_cg_c = -z_cg_c;
end

x_cg_c = [0;0;1] - z_cg_c*(z_cg_c.'*[0;0;1]);
x_cg_c = x_cg_c/norm(x_cg_c);
y_cg_c = cross(z_cg_c, x_cg_c);

R_c_cg = [x_cg_c y_cg_c z_cg_c];
rvec_c_cg = rodrigues(R_c_cg);
rvec_cg_c = -rvec_c_cg;

if nargout == 2
    tvec_cg_c = [0;0;dist_g_c];
end

end

