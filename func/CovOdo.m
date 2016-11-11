function [ mat_cov ] = CovOdo( ps2d_b1_b2, errConfig )
%COVODO Summary of this function goes here
%   Detailed explanation goes here

stdErrRatioOdoLin = errConfig.stdErrRatioOdoLin;
stdErrRatioOdoRot = errConfig.stdErrRatioOdoRot;
MinStdErrOdoLin = errConfig.MinStdErrOdoLin;
MinStdErrOdoRot = errConfig.MinStdErrOdoRot;

std_trans = max(norm(ps2d_b1_b2(1:2))*stdErrRatioOdoLin, MinStdErrOdoLin);
std_rot = max(abs(ps2d_b1_b2(3))*stdErrRatioOdoRot, MinStdErrOdoRot);

mat_std = diag([std_trans;std_trans;std_rot]);
mat_cov = mat_std*mat_std;

end

