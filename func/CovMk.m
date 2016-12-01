function [ mat_cov ] = CovMk( tvec_c_m, config_error )
%COVMK Summary of this function goes here
%   Detailed explanation goes here

stdErrRatioMkX = config_error.mk.stdratio_x;
stdErrRatioMkY = config_error.mk.stdratio_y;
stdErrRatioMkZ = config_error.mk.stdratio_z;

dist = norm(tvec_c_m);
std_z_c = max(dist*stdErrRatioMkZ, 0.001);
std_x_c = max(dist*stdErrRatioMkX, 0.001);
std_y_c = max(dist*stdErrRatioMkY, 0.001);
mat_std = diag([std_x_c;std_y_c;std_z_c]);
mat_cov = mat_std*mat_std;

end

