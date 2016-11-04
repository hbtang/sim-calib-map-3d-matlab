classdef ClassSolver
    %CLASSSOLVER class of solver for calibration and mapping
    
    properties
        % figure handle for display estimation resolt
        hdFigSolver;
        % err model
        errConfig;
    end
    
    methods
        
        % constructor function
        function this = ClassSolver(errConfig)
            if nargin == 0
                this.errConfig.stdErrRatioOdoLin = 0.01;
                this.errConfig.stdErrRatioOdoRot = 0.002;
                this.errConfig.stdErrRatioMkX = 0.002;
                this.errConfig.stdErrRatioMkY = 0.002;
                this.errConfig.stdErrRatioMkZ = 0.01;
            elseif nargin == 1
                this.errConfig = errConfig;
            end
        end
        
        %% solver step 1: estimate ground plane in camera frame
        % to obtain pvec_g_c and dist_g_c
        SolveGrndPlane(this, measure, calib);
        cost = CostGrndPlane(this, q, mk);
        DrawResGrndPlane(this, measure, calib);
        % solve ground plane from linear constraints
        SolveGrndPlaneLin(this, measure, calib);
        
        %% solve step 2: generate initial guess of full calibration problem,
        % according to local observation, no initial guess needed in this
        % step.
        SolveLocalOpt(this, measure, calib);
        [ vecCost, matJacobian ] = CostLocalOpt(this, q, measure, calib);
        
        % alternative solution of step 2: local optimization with loop
        % closing info, but no slam
        SolveLocalLoopOpt(this, measure, calib);
        [ vecCost, matJacobian ] = CostLocalLoopOpt(this, q, measure, calib);
        
        % another solution of step 2: with linear constraints, no iterative
        % optimization.
        SolveYawXY(this, measure, calib);
        
        
        %% solver step 3: solve full calibration by joint optimization
        % consider 3 dof extrinsic in ground plane ps2d_b_cg
        SolveJointOpt(this, measure, calib, map);
        [vecCost, matJacobian] = CostJointOpt(this, q, mk, odo, calib);
        
        % consider 5 dof extrinsic except z_b_c = 0
        SolveJointOpt2(this, measure, calib, map);
        [vecCost, matJacobian] = CostJointOpt2(this, q, mk, odo, calib);
        
        % consider 5 dof extrinsic and 1 dof time delay
        SolveJointOpt3(this, measure, calib, map);
        [vecCost, matJacobian] = CostJointOpt3(this, q, mk, odo, time, calib);
        
        %% solve SLAM only, fixed on given calib
        SolveSlam(this, measure, calib, map);
        [vecCost, matJacobian] = CostSlam(this, q, mk, odo, time, calib);
        
    end
    
end

