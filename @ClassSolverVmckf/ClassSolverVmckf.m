classdef ClassSolverVmckf  < handle
    % ClassSolverVmckfSw: calibration solver with VMCKF and sliding-window
    
    %% variables
    properties
        % state vector and covariance matrix: parameters to be calibrated
        vec_mu_x;
        mat_Sigma_x;
        
        % sliding window: struct with data
        struct_slidingWindow;
        
        % error configure
        errConfig;
        
        % status
        lpNow;
        
    end
    
    %% functions
    methods
        function this = ClassSolverVmckf(errConfig)
            % construct function
            this.errConfig = errConfig;
            this.struct_slidingWindow.vec_lp_sw = [];
            this.lpNow = [];
        end
        
        %% io
        % read from or set to calib of state vec_mu_x
        SetStateFromCalib(this, calib, flag, bInitCov);
        SetCalibFromState(this, calib);
        
        %% sliding window
        % clear sliding window
        function ClearSlidingWindow(this)
            this.struct_slidingWindow.vec_lp_sw = [];
        end        
        % renew sliding window
        struct_swNew = RenewSlidingWindow(this, lp_now, measure, struct_sw);
        
        %% create batch and constraint
        % create batch, batch.vec_lp loops when batch.id is observed
        cell_batch = CreateBatch(this, lp_now, measure, struct_sw);
        cell_batch = CreateBatchPair(this, lp_now, measure, struct_sw);
        % create constraint, all info of each constraint
        cell_cnstr = CreateCnstr(this, batch, measure);        
        % propagate odometry covariance
        PropagateCovOdo(this, lp_now, measure);
        
        %% core steps
        
        % measurement
        [vec_mu_y, mat_Sigma_y] = Measure(this, struct_cnstr);
        
        % compute Hessian, Jacobian and Cost
        % case 0: spatio only
        [vec_Cost, mat_Jacobian, cellmat_Hessian] = CostJacobHess_Spatio(this, vec_mu_x, vec_mu_y);
        [vec_Cost] = Cost_Spatio(this, vec_mu_x, vec_mu_y);
        % case 1: spatio-temporal       
        [vec_Cost, mat_Jacobian, cellmat_Hessian] = CostJacobHess_SpatioTemporal(this, struct_cnstr, vec_mu_x, vec_mu_y);
        % case 2: spatio-odo
        [vec_Cost, mat_Jacobian, cellmat_Hessian] = CostJacobHess_SpatioOdo(this, vec_mu_x, vec_mu_y);
        [vec_Cost] = Cost_SpatioOdo(this, vec_mu_x, vec_mu_y);
        % old functions
        [vec_Cost, mat_Jacobian, cellmat_Hessian] = CostApproxCnstr(this, struct_cnstr, vec_mu_x, vec_mu_y);
                
        % create virtual measurement
        [vec_mu_z, mat_Sigma_z] = VirtualMeasure(this, mat_Sigma_x, mat_Sigma_y, cellmat_Hessian);
        
        % do correct mu_x and sigma_x
        [vec_mu_x_new, mat_Sigma_x_new] = Correct(this, ...
            vec_mu_x, mat_Sigma_x, vec_mu_y, mat_Sigma_y, vec_mu_z, mat_Sigma_z, vec_Cost, mat_Jacobian);
        
        % refine covariance for over estimation
        [mat_Sigma_x_ref] = Refine(this, flag, vec_mu_x, vec_mu_x_new, mat_Sigma_x_new);
        
    end
    
end

