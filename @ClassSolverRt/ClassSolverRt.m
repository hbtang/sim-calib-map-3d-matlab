classdef ClassSolverRt  < handle
    %CLASSRTSOLVER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % measurement data
        odoNew; mkNew; odoRec; mkRec;
        
        % calibration data
        % plane vector of ground in camera frame, and its covariance mat
        pl_g_c; sigma_pl_g_c;
        
        % full calibration: estimation results for online calibration
        % quaternion vector of rotation from base to camera, 4 dof, norm=1
        q_c_b; sigma_q_c_b;
        % 3d location of camera origin in base
        pt3_c_b; sigma_pt3_c_b;
        % full calibration results: 6 dof [qx;qy;qz;qw;x;y]
        x_full; sigma_x_full;
        
        % for temporal calibration
        dt; x_full_dt; sigma_x_full_dt;
        
        % error configure: including error ration of odometry and mark
        % observation
        errConfig;
        
        % vec_lp_sw: vector of loop index belong to current sliding window
        vec_lp_sw;
        
    end
    
    methods
        function this = ClassSolverRt()
            % init error configuration
            % for agv experiment dataset
            % this.errConfig.errRatioOdoTr = 0.03;
            % this.errConfig.errRatioOdoRot = 0.05;
            % this.errConfig.errRatioMkDpt = 0.01;
            % this.errConfig.errRatioMkLtr = 0.002;
            
            this.errConfig.errRatioOdoTr = 0.01;
            this.errConfig.errRatioOdoRot = 0.01;
            this.errConfig.errRatioMkDpt = 0.01;
            this.errConfig.errRatioMkLtr = 0.01;
            
            % init record data
            this.mkNew = struct('lp', [], 'id', [], ...
                'rvec', [], 'tvec', [], 'num', [], ...
                'numMkId', [], 'vecMkId', []);
            this.odoNew = struct('lp',[], 'x',[],'y',[],'theta',[], ...
                'num', [], 'vx', [], 'vy', [], 'vtheta', []);
            this.mkRec = struct('lp', [], 'id', [], ...
                'rvec', [], 'tvec', [], 'num', [], ...
                'numMkId', [], 'vecMkId', []);
            this.odoRec = struct('lp',[], 'x',[],'y',[],'theta',[], ...
                'num', [], 'vx', [], 'vy', [], 'vtheta', []);
            this.odoRec.sigma = cell(0,1);
            
            % init estimation results of ground plane calibration
            this.pl_g_c = [0;1;0];
            this.sigma_pl_g_c = diag([1e-3;1e-3;1e-3]);
            
            % init estimation results of full calibration
            this.q_c_b = [0;0;1/sqrt(2);-1/sqrt(2)];
            this.sigma_q_c_b = diag([3;3;3;3]);
            this.pt3_c_b = [1000;1000;0];
            this.sigma_pt3_c_b = diag([1e6;1e6;1e6]);
            this.x_full = [this.q_c_b; this.pt3_c_b(1:2)];
            this.sigma_x_full = blkdiag(this.sigma_q_c_b, this.sigma_pt3_c_b(1:2,1:2));
            
            % init sliding window
            this.vec_lp_sw = [];
            
            % init temporal data
            this.dt = 0;
            this.x_full_dt = [this.q_c_b; this.pt3_c_b(1:2); this.dt];
            this.sigma_x_full_dt = blkdiag(this.sigma_q_c_b, this.sigma_pt3_c_b(1:2,1:2), 1);
            
        end
        
        % function renew measurements
        SetMeasNew(this, odo, mk);
        RenewMeasRec(this, bUseVel);
        
        % obtain z according to chi-square, by Hessian H and Covariance C
        [vecMu_z, matSigma_z] = FuncChiSqr(this, H, C);
        
        % propagate covariance matrix of odometry
        [odoNew_out] = FunPrpgOdo(this, odoLast, odoNew);
        
        %% ground plane calibration
        % function estimate ground plane
        CalibGrnd(this);
        
        % create measurements Y in ground plane calibration
        [mu_dpt, sigma_dpt] = CreateYGrnd(this, pt1, pt2)
        
        % create virtual measurement Z in ground plane calibration
        [mu_z, sigma_z, jacobian_z] = CreateZGrnd(this, ...
            mu_x, sigma_x, mu_y, sigma_y);
        
        % correct XY based on Z in ground plane calibration
        [mu_xy_corr, sigma_xy_corr] = CorrectXYGrnd( this, mu_x, sigma_x, ...
            mu_y, sigma_y, mu_z, sigma_z, jacobian_z );
        
        %% full estimation
        % function estimate camera extrinsic parameters
        CalibFull(this);
        
        % create measurements Y in full calibration
        [mu_y, sigma_y] = CreateYFull(this, rowNew, rowRec);
        
        % create virtual measurements Z in full calibration
        [mu_z, sigma_z, jacobian_z] = CreateZFull(this, mu_x, sigma_x, mu_y, sigma_y);
        
        % correct XY based on Z in full calibration
        [mu_xy_corr, sigma_xy_corr] = CorrectXYFull(this, ...
            mu_x, sigma_x, mu_y, sigma_y, mu_z, sigma_z, jacobian_z);
        
        % set initial guess for full calibration
        InitXFull(this, q_c_b, sigma_q_c_b, pt3_c_b, sigma_pt3_c_b);
        
        %% for temporal calibration
        CalibFullDelay(this);
        
        [ mu_y, sigma_y, odoinfo ] = CreateYFullDelay( this, rowNew, rowRec );
        
        [mu_z, sigma_z, jacobian_z] = CreateZFullDelay(this, mu_x, sigma_x, mu_y, sigma_y, odoinfo);
        
        [mu_xy_corr, sigma_xy_corr] = CorrectXYFullDelay(this, ...
            mu_x, sigma_x, mu_y, sigma_y, mu_z, sigma_z, jacobian_z, odoinfo);
        
        
        %% for sliding window
        % refresh vec_lp_sw with current measurement
        RefreshSlidingWindow(this, lp_now, measure);
        cell_batch = CreateBatch(this, lp_now, measure);
        
    end
    
end

