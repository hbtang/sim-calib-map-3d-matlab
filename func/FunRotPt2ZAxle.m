function [ R3d_c_c2 ] = FunRotPt2ZAxle( pt3d_c_m )
%FUNROT2CENTER generate a rotation matrix R3d_c_c2, from camera frame c to
% a new camera frame c2, so that the point m in c2 is on its z axle

if(pt3d_c_m(3) < 0)
    error('error: point on the negative part of camera z axis!');
end

nvec_m = pt3d_c_m/norm(pt3d_c_m);
nvec_z = [0;0;1];

vec_rot = cross(nvec_z,nvec_m);
nvec_rot = vec_rot/norm(vec_rot);
theta_c_c2 = asin(norm(vec_rot));

rvec_c_c2 = nvec_rot*theta_c_c2;
R3d_c_c2 = rodrigues(rvec_c_c2);

end

