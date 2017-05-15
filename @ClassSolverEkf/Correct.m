function Correct(this, struct_measure)
%CORRECT 此处显示有关此函数的摘要
%   此处显示详细说明

%% read mk measure
mk = struct_measure.mk;
num_mk = numel(mk.id);
if num_mk == 0
    return;
end

%% read error config
stdratio_x = this.err_config.mk.stdratio_x;
stdratio_y = this.err_config.mk.stdratio_y;
stdratio_z = this.err_config.mk.stdratio_z;

%% read current estimation
vec_mu_x = this.vec_mu_x;
mat_Sigma_x = this.mat_Sigma_x;
dim_x = numel(vec_mu_x);

%% do correct
% consider observation model: z = h(x) + v = H(x-x0) + v

H = zeros(3*num_mk, dim_x);
z = zeros(3*num_mk, 1);
z_pred = zeros(3*num_mk, 1);
mat_Sigma_z = zeros(3*num_mk, 3*num_mk);

rvec_b_c = vec_mu_x(1:3);
tvec_b_c = vec_mu_x(4:6);
se2_w_b = vec_mu_x(7:9);

for i = 1:num_mk
    % read info
    mkid = mk.id(i);
    tvec_c_m = mk.tvec(i,:).';
    
    % compute index
    row_vecmkid = find(this.vec_mkid == mkid, 1);
    rows_tveccm_x = (9+row_vecmkid*3-2: 9+row_vecmkid*3);
    tvec_w_m = vec_mu_x(rows_tveccm_x);
    
    % compute Jacobian
    H_rvecbc = JacobianNum(@(x)this.FunTveccm(x, tvec_b_c, se2_w_b, tvec_w_m), rvec_b_c);
    H_tvecbc = JacobianNum(@(x)this.FunTveccm(rvec_b_c, x, se2_w_b, tvec_w_m), tvec_b_c);
    H_se2wb = JacobianNum(@(x)this.FunTveccm(rvec_b_c, tvec_b_c, x, tvec_w_m), se2_w_b);
    H_tvecwm = JacobianNum(@(x)this.FunTveccm(rvec_b_c, tvec_b_c, se2_w_b, x), tvec_w_m);
    
    % compute z and mat_Sigma_z
    z(3*i-2:3*i,:) = tvec_c_m;
    z_pred(3*i-2:3*i,:) = this.FunTveccm(rvec_b_c, tvec_b_c, se2_w_b, tvec_w_m);
    depth_c_m = tvec_c_m(3);
    mat_std = diag([depth_c_m*stdratio_x; depth_c_m*stdratio_y; depth_c_m*stdratio_z]);
    mat_cov = mat_std * mat_std;
    mat_Sigma_z(3*i-2:3*i, 3*i-2:3*i) = mat_cov;
    
    % fill in block of C
    H(3*i-2:3*i, 1:3) = H_rvecbc;
    H(3*i-2:3*i, 4:6) = H_tvecbc;
    H(3*i-2:3*i, 7:9) = H_se2wb;
    H(3*i-2:3*i, rows_tveccm_x) = H_tvecwm;
end

K = mat_Sigma_x * H.' / (H*mat_Sigma_x*H.' + mat_Sigma_z);
vec_mu_x_bar = vec_mu_x + K*(z - z_pred);
mat_Sigma_x_bar = (eye(dim_x) - K*H) * mat_Sigma_x;

mat_Sigma_x_bar = (mat_Sigma_x_bar + mat_Sigma_x_bar.')/2;


%% refine d_mu
d_rvec_max = 0.1;
d_tvec_max = 200;
vec_d_mu_x = vec_mu_x_bar - vec_mu_x;
for i = 1:3
    if abs(vec_d_mu_x(i)) > d_rvec_max
        vec_d_mu_x(i) = d_rvec_max * sign(vec_d_mu_x(i));
    end
end
for i = 4:6
    if abs(vec_d_mu_x(i)) > d_tvec_max
        vec_d_mu_x(i) = d_tvec_max * sign(vec_d_mu_x(i));
    end
end
vec_mu_x_bar = vec_d_mu_x + vec_mu_x;

%% refine cov
% refine the covariance, when large offset happens
% spatio only, mu_x = [rvec_b_c; tvec_b_c]


mat_Sigma_r = mat_Sigma_x_bar(1:3, 1:3);
mat_Sigma_t = mat_Sigma_x_bar(4:6, 4:6);

dist_r = norm(vec_d_mu_x(1:3));
eig_r = sqrt(eig(mat_Sigma_r));
if dist_r > max(eig_r)*1.5 && dist_r > 0.03 && min(eig_r) < 3
    mat_Sigma_temp = (dist_r)^2*eye(3);
    mat_Sigma_x_bar(1:3,1:3) = mat_Sigma_r + mat_Sigma_temp;
end

dist_t = norm(vec_d_mu_x(4:6));
eig_t = sqrt(eig(mat_Sigma_t));
if dist_t > max(eig_t)*3 && dist_t > 100 && min(eig_t) < 300
    mat_Sigma_temp = (dist_t)^2*eye(3);
    mat_Sigma_x_bar(4:6,4:6) = mat_Sigma_t + mat_Sigma_temp;
end

mat_Sigma_x_bar(1:3,1:3) = RefineCov(mat_Sigma_x_bar(1:3,1:3), 1e-6);
mat_Sigma_x_bar(4:6,4:6) = RefineCov(mat_Sigma_x_bar(4:6,4:6), 1);

%% output
this.vec_mu_x = vec_mu_x_bar;
this.mat_Sigma_x = mat_Sigma_x_bar;

%% debug
% d_z = z - z_pred;
% d_vec_mu_x = vec_mu_x_bar - vec_mu_x;
% 
% if(abs(d_vec_mu_x(1)) > 0.03)
%     debug = 1;
% end
% 
% mat_Sigma_x_bar(1:3,1:3)
% mat_Sigma_x_bar(4:5,4:5)

% d_vec_mu_x(1:6)

% eig(mat_Sigma_x_bar);

end

