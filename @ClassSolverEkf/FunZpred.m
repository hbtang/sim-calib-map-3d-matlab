function [z_pred, z, dz] = FunZpred(this, vec_mu_x, vec_mkid, struct_measure)
%FUNZPRED 此处显示有关此函数的摘要
%   此处显示详细说明

mk = struct_measure.mk;
num_mk = numel(mk.id);

vec_mu_x = vec_mu_x;

z = zeros(3*num_mk, 1);
z_pred = zeros(3*num_mk, 1);

rvec_b_c = vec_mu_x(1:3);
tvec_b_c = vec_mu_x(4:6);
se2_w_b = vec_mu_x(7:9);

for i = 1:num_mk
    % read info
    mkid = mk.id(i);
    tvec_c_m = mk.tvec(i,:).';
    
    % compute index
    row_vecmkid = find(vec_mkid == mkid, 1);
    rows_tveccm_x = (9+row_vecmkid*3-2: 9+row_vecmkid*3);
    tvec_w_m = vec_mu_x(rows_tveccm_x);
        
    % compute z and mat_Sigma_z
    z(3*i-2:3*i,:) = tvec_c_m;
    z_pred(3*i-2:3*i,:) = this.FunTveccm(rvec_b_c, tvec_b_c, se2_w_b, tvec_w_m);
end

dz = z_pred - z;

end

