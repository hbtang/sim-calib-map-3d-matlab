classdef ClassSolverEkfRelMk < handle
    %CLASSSOLVEREKF 此处显示有关此类的摘要
    %   此处显示详细说明
    
    %% variables
    properties
        % state vector and covariance matrix: parameters to be calibrated
        vec_mu_x;
        mat_Sigma_x;
        
        % tvec_c_m, w.r.t. the init kf of each mk
        vec_mk_id;
        vec_mk_rowbeg;
        vec_mk_rowend;
        vec_mk_lpinit;
        
        % se2_w_b, of the init kfs
        vec_kf_lp;
        vec_kf_rowbeg;
        vec_kf_rowend;
        
        % error configure
        err_config;
        
        % status
        lp_now;
        lp_last;
    end
    
    %% functions
    methods
        function this = ClassSolverEkfRelMk(err_config)
            % construct function
            this.err_config = err_config;
            
            this.lp_now = [];
            this.lp_last = [];
            this.vec_mu_x = [];
            this.mat_Sigma_x = [];
            
            this.vec_mk_id = [];
            this.vec_mk_rowbeg = [];
            this.vec_mk_rowend = [];
            this.vec_mk_lpinit = [];
            
            this.vec_kf_lp = [];
            this.vec_kf_rowbeg = [];
            this.vec_kf_rowend = [];
        end
        
        function rvec_b_c = GetRvec(this)
            rvec_b_c = this.vec_mu_x(1:3);
        end
        function tvec_b_c = GetTvec(this)
            tvec_b_c = this.vec_mu_x(4:6);
        end
        
        %% init
        % read from or set to calib of state vec_mu_x
        SetStateFromCalib(this, calib, flag, b_initcov);
        InitBasePose(this, measure);
        
        %% read
        [b_read_fail, struct_measure] = ReadMeasure(this, measure);
        [struct_measure] = InitMkNew(this, struct_measure);
        [struct_measure] = ClearMkOld(this, struct_measure);
        
        %% propagation
        % propagate odometry covariance
        Propagate(this, struct_measure);
        
        %% correction
        Correct(this, struct_measure);
        
        %% member functions
        tvec_w_m = FunTvecwm(this, rvec_b_c, tvec_b_c, se2_w_b, tvec_c_m);
        tvec_c_m = FunTveccm(this, rvec_b_c, tvec_b_c, se2_w_b, tvec_w_m);
        [z_pred, z, dz] = FunZpred(this, vec_mu_x, vec_mkid, struct_measure);
        tvec_c_m = FunTveccmRelMk(this, rvec_b_c, tvec_b_c,...
            se2_w_b, se2_w_bkf, tvec_ckf_m);
        
    end
    
end

