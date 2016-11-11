function [vec_mu_x_new, mat_Sigma_x_new] = ...
    Correct(this, vec_mu_x, mat_Sigma_x, vec_mu_y, mat_Sigma_y, vec_mu_z, mat_Sigma_z, Cost, Jacobian)
%CORRECT Summary of this function goes here
%   Detailed explanation goes here

sigma_xy = blkdiag(mat_Sigma_x, mat_Sigma_y);
jacobian_z = Jacobian;
mu_xy = [vec_mu_x; vec_mu_y];
f_mu = Cost;
mu_z = vec_mu_z;
sigma_z = mat_Sigma_z;

num_x = numel(vec_mu_x);
num_y = numel(vec_mu_y);
num_xy = num_x + num_y;

%% generate information matrix omega
omega_1 = inv(sigma_xy);
omega_2 = jacobian_z.'*inv(sigma_z)*jacobian_z;
xi_1 = zeros(num_xy,1);
xi_2 = ((-f_mu-mu_z).'*inv(sigma_z)*jacobian_z).';

omega_dxy_corr = omega_1+omega_2;
xi_dxy_corr = xi_1+xi_2;

sigma_dxy_corr = inv(omega_dxy_corr);
mu_dxy_corr = sigma_dxy_corr*xi_dxy_corr;

mu_xy_corr = mu_dxy_corr + mu_xy;
sigma_xy_corr = sigma_dxy_corr;

%% return
vec_mu_x_new = mu_xy_corr(1:num_x);
mat_Sigma_x_new = sigma_xy_corr(1:num_x, 1:num_x);

end

