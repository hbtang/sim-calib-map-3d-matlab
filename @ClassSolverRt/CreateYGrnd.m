function [mu_dpt, sigma_dpt] = CreateYGrnd(this, pt1, pt2)
%FUNCGDDPT function: generate mean and covariance of dpt = pt1-pt2

mu_dpt = pt1-pt2;

stdRatioErrZ = this.errConfig.errRatioMkDpt;
stdRatioErrXY = this.errConfig.errRatioMkLtr;

depth_pt1 = abs(pt1(3));
std_pt1_x = stdRatioErrXY*depth_pt1;
std_pt1_y = stdRatioErrXY*depth_pt1;
std_pt1_z = stdRatioErrZ*depth_pt1;
sigma_pt1 = diag([std_pt1_x^2; std_pt1_y^2; std_pt1_z^2]);

depth_pt2 = abs(pt2(3));
std_pt2_x = stdRatioErrXY*depth_pt2;
std_pt2_y = stdRatioErrXY*depth_pt2;
std_pt2_z = stdRatioErrZ*depth_pt2;
sigma_pt2 = diag([std_pt2_x^2; std_pt2_y^2; std_pt2_z^2]);

sigma_dpt = sigma_pt1+sigma_pt2;

end

