function [ mat_cov ] = CovMk( tvec_c_m, errConfig )
%COVMK Summary of this function goes here
%   Detailed explanation goes here

stdErrRatioMkX = errConfig.stdErrRatioMkX;
stdErrRatioMkY = errConfig.stdErrRatioMkY;
stdErrRatioMkZ = errConfig.stdErrRatioMkZ;

dist = norm(tvec_c_m);
std_z_c = max(dist*stdErrRatioMkZ, 0.001);
std_x_c = max(dist*stdErrRatioMkX, 0.001);
std_y_c = max(dist*stdErrRatioMkY, 0.001);
mat_std = diag([std_x_c;std_y_c;std_z_c]);
mat_cov = mat_std*mat_std;

end

