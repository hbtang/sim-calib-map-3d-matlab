function [ vec_cost ] = Cost_SpatioOdo( this, vec_mu_x, vec_mu_y )
%COST_SPATIOODO Summary of this function goes here
%   Detailed explanation goes here

rvec_b_c = vec_mu_x(1:3);
R3_b_c = rodrigues(rvec_b_c);
tvec_b_c = [vec_mu_x(4:5); 0];

k_odo_lin = vec_mu_x(6);
k_odo_rot = vec_mu_x(7);

x_b1_b2_raw = vec_mu_y(1); y_b1_b2_raw = vec_mu_y(2); theta_b1_b2_raw = vec_mu_y(3);
x_b1_b2 = x_b1_b2_raw*k_odo_lin; y_b1_b2 = y_b1_b2_raw*k_odo_lin;
tvec_b1_b2 = [x_b1_b2; y_b1_b2; 0];
theta_b1_b2 = theta_b1_b2_raw*k_odo_rot;
rvec_b1_b2 = [0;0;theta_b1_b2];
R3_b1_b2 = rodrigues(rvec_b1_b2);

tvec_c1_m = vec_mu_y(4:6);
tvec_c2_m = vec_mu_y(7:9);

vec_cost = R3_b_c*tvec_c1_m + tvec_b_c - R3_b1_b2*R3_b_c*tvec_c2_m - R3_b1_b2*tvec_b_c - tvec_b1_b2;

end

