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
% consider observation model: z = Cx + v

C = zeros(3*num_mk, dim_x);
z = zeros(3*num_mk, 1);
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
    
    % compute z and mat_Sigma_z
    z(3*i-2:3*i,:) = tvec_c_m;
    z_c_m = tvec_c_m(3);
    mat_std = diag([z_c_m*stdratio_x; z_c_m*stdratio_y; z_c_m*stdratio_z]);
    mat_cov = mat_std * mat_std;
    mat_Sigma_z(3*i-2:3*i, 3*i-2:3*i) = mat_cov;
    
    % compute Jacobian
    C_rvecbc = JacobianNum(@(x)this.FunTveccm(x, tvec_b_c, se2_w_b, tvec_w_m), rvec_b_c);
    C_tvecbc = JacobianNum(@(x)this.FunTveccm(rvec_b_c, x, se2_w_b, tvec_w_m), tvec_b_c);
    C_se2wb = JacobianNum(@(x)this.FunTveccm(rvec_b_c, tvec_b_c, x, tvec_w_m), se2_w_b);
    C_tvecwm = JacobianNum(@(x)this.FunTveccm(rvec_b_c, tvec_b_c, se2_w_b, x), tvec_w_m);
    
    % fill in block of C
    C(3*i-2:3*i, 1:3) = C_rvecbc;
    C(3*i-2:3*i, 4:6) = C_tvecbc;
    C(3*i-2:3*i, 7:9) = C_se2wb;
    C(3*i-2:3*i, rows_tveccm_x) = C_tvecwm;
end

K = mat_Sigma_x * C.' / (C*mat_Sigma_x*C.' + mat_Sigma_z);
vec_mu_x_bar = vec_mu_x + K*(z - C*vec_mu_x);
mat_Sigma_x_bar = (eye(dim_x) - K*C) * mat_Sigma_x;

%% output
this.vec_mu_x = vec_mu_x_bar;
this.mat_Sigma_x = mat_Sigma_x_bar;

end

