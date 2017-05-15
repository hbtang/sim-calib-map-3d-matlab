function [ struct_measure ] = InitMkNew(this, struct_measure)
%INITMKNEW 此处显示有关此函数的摘要
%   此处显示详细说明




num_mk = numel(struct_measure.mk.id);

vec_mu_x = this.vec_mu_x;
mat_Sigma_x = this.mat_Sigma_x;
vec_mkid = this.vec_mkid;

rvec_b_c = vec_mu_x(1:3);
tvec_b_c = vec_mu_x(4:6);
se2_w_b = vec_mu_x(7:9);

mat_Sigma_x9 = mat_Sigma_x(1:9,1:9);

rows_mk_keep = [];
vec_mu_x_bar = vec_mu_x;
mat_Sigma_x_bar = mat_Sigma_x;
for i = 1:num_mk
    %% if already initialized
    mkid = struct_measure.mk.id(i);
    if any(this.vec_mkid == mkid)
        rows_mk_keep = [rows_mk_keep; i];
        continue;
    end
    tvec_c_m = struct_measure.mk.tvec(i,:).';
    
    %% renew vec_mkid
    vec_mkid = [vec_mkid; mkid];
    
    %% compute tvec_w_m, and renew vec_mu_x    
    tvec_w_m = this.FunTvecwm(rvec_b_c, tvec_b_c, se2_w_b, tvec_c_m);
    vec_mu_x_bar = [vec_mu_x_bar; tvec_w_m];
    
    %% compute Jacobian
    J_tvecwm_rvecbc = JacobianNum(@(x)this.FunTvecwm(x, tvec_b_c, se2_w_b, tvec_c_m), rvec_b_c);
    J_tvecwm_tvecbc = JacobianNum(@(x)this.FunTvecwm(rvec_b_c, x, se2_w_b, tvec_c_m), tvec_b_c);
    J_tvecwm_se2wb = JacobianNum(@(x)this.FunTvecwm(rvec_b_c, tvec_b_c, x, tvec_c_m), se2_w_b);    
    % Jacobian with the first 9 elements of vec_mu_x
    J_tvecwm_x9 = [J_tvecwm_rvecbc J_tvecwm_tvecbc J_tvecwm_se2wb];
    
    %% compute cov and renew mat_Sigma_x
    mat_Sigma_tvecwm_tvecwm = J_tvecwm_x9 * mat_Sigma_x9 * J_tvecwm_x9.';
    mat_Sigma_x9_tvecwm = J_tvecwm_x9 * mat_Sigma_x9;
%     mat_Sigma_tvecwm_x9 = mat_Sigma_x9 * J_tvecwm_x9.';
    
    mat_Sigma_x_bar = blkdiag(mat_Sigma_x_bar, mat_Sigma_tvecwm_tvecwm);
%     mat_Sigma_x_bar(end-2:end, 1:9) = mat_Sigma_x9_tvecwm;
%     mat_Sigma_x_bar(1:9, end-2:end) = mat_Sigma_x9_tvecwm.';
    
end

mat_Sigma_x_bar = (mat_Sigma_x_bar + mat_Sigma_x_bar.')/2;

%% output
this.vec_mu_x = vec_mu_x_bar;
this.mat_Sigma_x = mat_Sigma_x_bar;
this.vec_mkid = vec_mkid;

struct_measure.mk.id = struct_measure.mk.id(rows_mk_keep,:);
struct_measure.mk.rvec = struct_measure.mk.rvec(rows_mk_keep,:);
struct_measure.mk.tvec = struct_measure.mk.tvec(rows_mk_keep,:);

end
