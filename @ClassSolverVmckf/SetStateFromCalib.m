function SetStateFromCalib( this, calib, flag, bInitCov )

if nargin < 4
    bInitCov = false;
end

% flag:
% 0 for spatio only 6 dof,
% 1 for spatio-temporal 7 dof, [q_b_c; x_b_c; y_b_c; dt]
% 2 for spatio-odometric 7 dof, [rvec_b_c; x_b_c; y_b_c; k_odo_lin; k_odo_rot]
switch flag
    case 0
        % 0 for spatio only 6 dof, [q_b_c; x_b_c; y_b_c]
        rvec_b_c = calib.rvec_b_c;
        tvec_b_c = calib.tvec_b_c;
        qvec_b_c = rodrigues2quat(rvec_b_c);
        
        this.vec_mu_x = zeros(6,1);
        this.vec_mu_x(1:4) = qvec_b_c;
        this.vec_mu_x(5:6) = tvec_b_c(1:2);
        
        if bInitCov
            this.mat_Sigma_x = blkdiag(eye(4)*3, eye(2)*1e6);
        end
        
    case 1
        % 1 for spatio-temporal 7 dof, [q_b_c; x_b_c; y_b_c; dt]
        rvec_b_c = calib.rvec_b_c;
        tvec_b_c = calib.tvec_b_c;
        qvec_b_c = rodrigues2quat(rvec_b_c);
        dt = calib.dt;
        
        this.vec_mu_x = zeros(7,1);
        this.vec_mu_x(1:4) = qvec_b_c;
        this.vec_mu_x(5:6) = tvec_b_c(1:2);
        this.vec_mu_x(7) = dt;
        
        if bInitCov
            this.mat_Sigma_x = blkdiag(eye(4)*3, eye(2)*1e6, 1e-4);
        end
        
    case 2
        % 2 for spatio-odometric 7 dof, [rvec_b_c; x_b_c; y_b_c; k_odo_lin; k_odo_rot]
        rvec_b_c = calib.rvec_b_c;
        tvec_b_c = calib.tvec_b_c;
        k_odo_lin = calib.k_odo_lin;
        k_odo_rot = calib.k_odo_rot;
        
        this.vec_mu_x = zeros(7,1);
        this.vec_mu_x(1:3) = rvec_b_c;
        this.vec_mu_x(4:5) = tvec_b_c(1:2);
        this.vec_mu_x(6) = k_odo_lin;
        this.vec_mu_x(7) = k_odo_rot;
        
        if bInitCov
            this.mat_Sigma_x = blkdiag(eye(3)*3, eye(2)*1e6, eye(2)*1e-2);
        end
        
    otherwise
end





end

