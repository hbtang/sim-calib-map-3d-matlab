classdef ClassSolverSlam
    %CLASSSOLVER class of solver for calibration and mapping
    
    properties
        % figure handle for display estimation resolt
        hdFigSolver;
        % error configuration
        setting;
    end
    
    methods
        
        % constructor function
        function this = ClassSolverSlam(settingInput)
            this.setting = settingInput;
        end
        
        %% calibration init. linear solution
        % solve ground plane by linear constraints
        SolveGrndPlaneLin(this, measure, calib);
        % solve YawXY by linear constraints
        SolveYawXY(this, measure, calib);
        
        %% calibration init. Guo's linear solution
        SolveInitGuo(this, measure, calib);
        % solve YawXY by Guo's
        SolveYawXYGuo(this, measure, calib);
        
        %% calibration init. non-linear solution
        %         % solve ground plane by non-linear constraints
        %         SolveGrndPlane(this, measure, calib);
        %         cost = CostGrndPlane(this, q, mk);
        %         % solve YawXY by non-linear constraints
        %         SolveLocalOpt(this, measure, calib);
        %         [ vecCost, matJacobian ] = CostLocalOpt(this, q, measure, calib);
        %         % solve YawXY by non-linear constraints, consider loop close
        %         SolveLocalLoopOpt(this, measure, calib);
        %         [ vecCost, matJacobian ] = CostLocalLoopOpt(this, q, measure, calib);
        
        %% calibration with joint optimization        
        % consider mark observation, do mark slam based calibration
        SolveJointOptMSlam(this, measure, calib, map, options);        
        % consider image feature only, do visual slam based calibration,
        SolveJointOptVSlam(this, measure, calib, map, options);
        
        %         [vecCost, matJacobian] = CostJointOptVSlam(this, q, mk, odo, time, calib, setting, options);
        
        %         % consider 3 dof extrinsic in ground plane ps2d_b_cg
        %         SolveJointOpt(this, measure, calib, map);
        %         [vecCost, matJacobian] = CostJointOpt(this, q, mk, odo, calib);
        %
        %         % consider 5 dof extrinsic except z_b_c = 0
        %         SolveJointOpt2(this, measure, calib, map);
        %         [vecCost, matJacobian] = CostJointOpt2(this, q, mk, odo, calib);
        %
        %         % consider 5 dof extrinsic and 1 dof time delay
        %         SolveJointOpt3(this, measure, calib, map);
        %         [vecCost, matJacobian] = CostJointOpt3(this, q, mk, odo, time, calib);
        %
        %         % consider 5 dof extrinsic, 1 dof time delay, 2 dof odometric
        %         SolveJointOpt4(this, measure, calib, map);
        %         [vecCost, matJacobian] = CostJointOpt4(this, q, mk, odo, time, calib);
        
        
        %% solve SLAM only, fixed on given calib
        %         % solve mark-SLAM problem, consider mark observation tvec_c_m
        %         SolveMSlam(this, measure, calib, map);
        %         [vecCost, matJacobian] = CostMSlam(this, q, mk, odo, time, calib);
        
        %% io
        % draw results gound plane
        %         DrawResGrndPlane(this, measure, calib, vec_ground);
        
    end
    
end

