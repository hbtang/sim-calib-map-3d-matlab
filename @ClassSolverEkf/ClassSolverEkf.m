classdef ClassSolverEkf < handle
    %CLASSSOLVEREKF 此处显示有关此类的摘要
    %   此处显示详细说明
    
    %% variables
    properties
        % state vector and covariance matrix: parameters to be calibrated
        vec_mu_x;
        mat_Sigma_x;
        vec_mkid;
        
        % error configure
        err_config;
        
        % status
        lp_now;
        lp_last;
    end
    
    %% functions
    methods
        function this = ClassSolverEkf(err_config)
            % construct function
            this.err_config = err_config;
            
            this.lp_now = [];
            this.lp_last = [];
            this.vec_mu_x = [];
            this.mat_Sigma_x = [];
            this.vec_mkid = [];
        end
        
        %% init
        % read from or set to calib of state vec_mu_x
        SetStateFromCalib(this, calib, flag, b_initcov);
        InitBasePose(this, measure);        
        
        %% read
        [b_read_fail, struct_measure] = ReadMeasure(this, measure);
        [struct_measure] = InitMkNew(this, struct_measure);
        
        %% propagation
        % propagate odometry covariance
        Propagate(this, struct_measure);
        
        %% correction
        Correct(this, struct_measure);
        
        %% output
        SetCalibFromState(this, calib);
        
        %% member functions
        tvec_w_m = FunTvecwm(this, rvec_b_c, tvec_b_c, se2_w_b, tvec_c_m);
        tvec_c_m = FunTveccm(this, rvec_b_c, tvec_b_c, se2_w_b, tvec_w_m);
        
    end
    
end

