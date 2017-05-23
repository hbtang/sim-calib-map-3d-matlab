function [ struct_measure ] = InitMkNew(this, struct_measure)
%INITMKNEW 此处显示有关此函数的摘要
%   此处显示详细说明
lp_now = struct_measure.lp_now;
num_mk = numel(struct_measure.mk.id);

%% load data from solver
vec_mu_x = this.vec_mu_x;
mat_Sigma_x = this.mat_Sigma_x;
vec_mk_id = this.vec_mk_id;
vec_mk_rowbeg = this.vec_mk_rowbeg;
vec_mk_rowend = this.vec_mk_rowend;
vec_mk_lpinit = this.vec_mk_lpinit;
vec_kf_lp = this.vec_kf_lp;
vec_kf_rowbeg = this.vec_kf_rowbeg;
vec_kf_rowend = this.vec_kf_rowend;
stdratio_x = this.err_config.mk.stdratio_x;
stdratio_y = this.err_config.mk.stdratio_y;
stdratio_z = this.err_config.mk.stdratio_z;

rvec_b_c = vec_mu_x(1:3);
tvec_b_c = vec_mu_x(4:6);
se2_w_b = vec_mu_x(7:9);

mat_Sigma_x9 = mat_Sigma_x(1:9,1:9);

%% init mk new
rows_mk_keep = [];
vec_mu_x_bar = vec_mu_x;
mat_Sigma_x_bar = mat_Sigma_x;
b_mk_init = false;
for i = 1:num_mk
    %% if already initialized
    mkid = struct_measure.mk.id(i);
    if any(vec_mk_id == mkid)
        rows_mk_keep = [rows_mk_keep; i];
        continue;
    end
    tvec_c_m = struct_measure.mk.tvec(i,:).';
    
    %% renew vec_mkid
    dim_vecmux_bar = numel(vec_mu_x_bar);
    vec_mk_id = [vec_mk_id; mkid];
    vec_mk_rowbeg = [vec_mk_rowbeg; dim_vecmux_bar + 1];
    vec_mk_rowend = [vec_mk_rowend; dim_vecmux_bar + 3];
    vec_mk_lpinit = [vec_mk_lpinit; lp_now];
    
    %% compute tvec_w_m, and renew vec_mu_x
    vec_mu_x_bar = [vec_mu_x_bar; tvec_c_m];
        
    %% compute cov and renew mat_Sigma_x    
    depth_c_m = tvec_c_m(3);
    mat_std = diag([depth_c_m*stdratio_x; depth_c_m*stdratio_y; depth_c_m*stdratio_z]);
    mat_Sigma_tveccm = mat_std * mat_std;
    mat_Sigma_x_bar = blkdiag(mat_Sigma_x_bar, mat_Sigma_tveccm);
    
    %% set flag
    b_mk_init = true;
end

%% init kf new
if b_mk_init
    dim_vecmux_bar = numel(vec_mu_x_bar);
    vec_kf_lp = [vec_kf_lp; lp_now];
    vec_kf_rowbeg = [vec_kf_rowbeg; dim_vecmux_bar+1];
    vec_kf_rowend = [vec_kf_rowend; dim_vecmux_bar+3];
    
    %% renew vec_mu_x_bar
    vec_mu_x_bar = [vec_mu_x_bar; se2_w_b];
    
    %% renew mat_Sigma_x_bar
    J_kf = [eye(dim_vecmux_bar); zeros(3, dim_vecmux_bar)];
    J_kf(end-2:end, 7:9) = eye(3);
    mat_Sigma_x_bar = J_kf * mat_Sigma_x_bar * J_kf.';
end

%% refine mat_Sigma
mat_Sigma_x_bar = (mat_Sigma_x_bar + mat_Sigma_x_bar.')/2;

%% renew solver
this.vec_mu_x = vec_mu_x_bar;
this.mat_Sigma_x = mat_Sigma_x_bar;

this.vec_mk_id = vec_mk_id;
this.vec_mk_rowbeg = vec_mk_rowbeg;
this.vec_mk_rowend = vec_mk_rowend;
this.vec_mk_lpinit = vec_mk_lpinit;
this.vec_kf_lp = vec_kf_lp;
this.vec_kf_rowbeg = vec_kf_rowbeg;
this.vec_kf_rowend = vec_kf_rowend;

%% delete mk init in struct_measure
struct_measure.mk.id = struct_measure.mk.id(rows_mk_keep,:);
struct_measure.mk.rvec = struct_measure.mk.rvec(rows_mk_keep,:);
struct_measure.mk.tvec = struct_measure.mk.tvec(rows_mk_keep,:);

end
