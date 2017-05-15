function Propagate(this, struct_measure)
%PROPAGATE 此处显示有关此函数的摘要
%   此处显示详细说明

%% read odo measure, compute cov
stdmin_lin = this.err_config.odo.stdmin_lin;
stdmin_rot = this.err_config.odo.stdmin_rot;
stdratio_lin = this.err_config.odo.stdratio_lin;
stdratio_rot = this.err_config.odo.stdratio_rot;

se2_b1_b2 = struct_measure.se2_b1_b2;
dist_b1_b2 = norm(se2_b1_b2(1:2));
theta_b1_b2 = se2_b1_b2(3);

std_lin = max(dist_b1_b2*stdratio_lin, stdmin_lin);
std_rot = max(abs(theta_b1_b2)*stdratio_rot, stdmin_rot);

mat_std = diag([std_lin; std_lin; std_rot]);
mat_Sigma_u = mat_std * mat_std;

%% do propagate
% consider motion model: x_bar = Ax + Bu

vec_mu_x = this.vec_mu_x;
mat_Sigma_x = this.mat_Sigma_x;
dim_x = numel(vec_mu_x);

% compute mean: vec_mu_x_bar
se2_w_b1 = vec_mu_x(7:9);
se2_w_b2 = FunMove2d(se2_w_b1, se2_b1_b2);
vec_mu_x_bar = vec_mu_x;
vec_mu_x_bar(7:9) = se2_w_b2;

% compute covariance: mat_Sigma_x_bar
[ ~, R_w_b1 ] = FunVec2Trans2d( se2_w_b1 );
A = eye(dim_x);
B_7_9 = blkdiag(R_w_b1, 1);
B = zeros(dim_x, 3);
B(7:9,:) = B_7_9;
mat_Sigma_x_bar = A*mat_Sigma_x*A.' + B*mat_Sigma_u*B.';

%% output
this.vec_mu_x = vec_mu_x_bar;
this.mat_Sigma_x = mat_Sigma_x_bar;

end

