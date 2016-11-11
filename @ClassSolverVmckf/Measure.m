function [vec_mu_y, mat_Sigma_y] = Measure(this, struct_cnstr)
%MEASURE Summary of this function goes here
%   Detailed explanation goes here

tvec_c1_m = struct_cnstr.tvec_c1_m;
tvec_c2_m = struct_cnstr.tvec_c2_m;
ps_b1_b2 = struct_cnstr.ps_b1_b2;

Sigma_ps_b1_b2 = struct_cnstr.Sigma_ps_b1_b2;
Sigma_tvec_c1_m = struct_cnstr.Sigma_tvec_c1_m;
Sigma_tvec_c2_m = struct_cnstr.Sigma_tvec_c2_m;

vec_mu_y = [ps_b1_b2; tvec_c1_m; tvec_c2_m];
mat_Sigma_y = max(blkdiag(Sigma_ps_b1_b2, Sigma_tvec_c1_m, Sigma_tvec_c2_m), eye(9)*1e-6);

end

