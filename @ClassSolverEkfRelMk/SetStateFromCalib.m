function SetStateFromCalib( this, calib, flag )
%SETSTATEFROMCALIB 此处显示有关此函数的摘要
%   此处显示详细说明

if nargin < 3
    flag = 0;
end
% if nargin < 4
%     b_initcov = true;
% end

switch flag
    case 0
        % mu_x = [rvec_b_c; tvec_b_c]
        rvec_b_c = calib.rvec_b_c;
        tvec_b_c = calib.tvec_b_c;
        this.vec_mu_x = [rvec_b_c; tvec_b_c];
        this.mat_Sigma_x = blkdiag(eye(3)*0.04, eye(3)*40000);      
    case 1
        % for debug
        rvec_b_c = calib.rvec_b_c;
        tvec_b_c = calib.tvec_b_c;
        this.vec_mu_x = [rvec_b_c; tvec_b_c];
        this.mat_Sigma_x = blkdiag(eye(3)*1e-4, eye(3)*1);  
    otherwise
end

end

