% main_vmckf_sw: run vmckf with sliding-window

%% init: create class objs and load data
clear;

% read configure file
b_sim = false;
if b_sim
    setting = YAML.read('setting-gf-sim.yml');
else
    setting = YAML.read('setting-gf-exp-d.yml');
end

% select record file
MdNs = 0; OdoNs = 0; SetId = 1;

for SetId = 1:20
    if b_sim
        % select record file
        stdRatioMkZ = 0.01*MdNs;
        stdRatioMkXY = 0.01*MdNs;
        stdRatioOdoL = 0.01*OdoNs;
        stdRatioOdoR = 0.01*OdoNs;
        NameMk = ['Mk-z', num2str(stdRatioMkZ*100), '-xy', num2str(stdRatioMkXY*100), ...
            '-s', num2str(SetId), '.rec'];
        NameOdo = ['Odo-l', num2str(stdRatioOdoL*100), '-r', num2str(stdRatioOdoR*100), ...
            '-s', num2str(1), '.rec'];
        if MdNs == 0
            NameMk = ['Mk-z', num2str(stdRatioMkZ*100), '-xy', num2str(stdRatioMkXY*100), ...
                '-s', num2str(1), '.rec'];
        end
        if OdoNs == 0
            NameOdo = ['Odo-l', num2str(stdRatioOdoL*100), '-r', num2str(stdRatioOdoR*100), ...
                '-s', num2str(1), '.rec'];
        end
        setting.path.markfilename = NameMk;
        setting.path.odofilename = NameOdo;
    end
    
    % read record data
    measure = ClassMeasure(setting.path.fold, setting.path.markfilename, setting.path.odofilename);
    measure.ReadRecData;
    
    %%%%%%%% Debug Begin %%%%%%%%
    % reset t_odo, because in AGV dataset bad match in t_odo
    % measure.time.t_odo = measure.time.t_mk;
    % measure.time.t_odo = TimeOdo_ConstVel(measure.time.t_odo);
    % measure.time.t_odo = TimeOdo_EqualOffset(measure.time.t_odo, measure.odo);
    %%%%%%%% Debug End  %%%%%%%%%
    
    measure_raw = ClassMeasure();
    measure.CopyTo(measure_raw);
    measure.PruneData(setting.prune.thresh_lin, setting.prune.thresh_rot);
    % compute velocity by interpolation
    measure.odo = Odo_Interpolate(measure.odo, measure_raw.odo, measure_raw.time, 1);
    measure_raw.odo = Odo_Interpolate(measure_raw.odo, measure_raw.odo, measure_raw.time, 1);
    
    % init calib
    calib = ClassCalib;
    tvec_b_c_init = setting.init.tvec_b_c.';
    rvec_b_c_init = setting.init.rvec_b_c.';
    rvec_b_c_init = normrnd([rvec_b_c_init], [0.3;0.3;0.3]);
    tvec_b_c_init = normrnd([tvec_b_c_init], [500; 500 ;0]);
    calib.SetVecbc(rvec_b_c_init, tvec_b_c_init);
    calib.dt = 0;
    calib.k_odo_lin = 1;
    calib.k_odo_rot = 1;
    
    % read error config, todo...
    errConfig.stdErrRatioOdoLin = setting.error.odo.stdratio_lin;
    errConfig.stdErrRatioOdoRot = setting.error.odo.stdratio_rot;
    errConfig.MinStdErrOdoLin = setting.error.odo.stdmin_lin;
    errConfig.MinStdErrOdoRot = setting.error.odo.stdmin_rot;
    errConfig.stdErrRatioMkX = setting.error.mk.stdratio_x;
    errConfig.stdErrRatioMkY = setting.error.mk.stdratio_y;
    errConfig.stdErrRatioMkZ = setting.error.mk.stdratio_z;
    
    % init ground truth
    tvec_b_c_true = setting.truth.tvec_b_c.';
    rvec_b_c_true = setting.truth.rvec_b_c.';
    
    % init solver
    % flag: 0 spatio, 1 spatio-temporal, 2 spatio-odo
    flag = 0;
    solver = ClassSolverVmckf(errConfig);
    solver.SetStateFromCalib(calib, flag, true);
    solver.ClearSlidingWindow;
    
    %% solve in rt, main loop
    vec_lp = measure.odo.lp;
    rec_mu_x = []; rec_cov_x = cell(0,0);
    
    rec_tvec = [];
    rec_rvec = [];
    rec_dtvec = [];
    rec_drvec = [];
    
    for i = 1:(numel(vec_lp))
        lp_now = vec_lp(i);
        
        % cell_batch = solver.CreateBatch(lp_now, measure, solver.struct_slidingWindow);
        cell_batch = solver.CreateBatchPair(lp_now, measure, solver.struct_slidingWindow);
        solver.PropagateCovOdo(lp_now, measure);
        
        if numel(cell_batch) ~= 0
            % for each batch: multi instants, same mark id
            for j = 1:numel(cell_batch)
                cell_cnstr = solver.CreateCnstr( cell_batch{j}, measure );
                
                vec_mu_x = solver.vec_mu_x;
                mat_Sigma_x = solver.mat_Sigma_x;
                
                vec_mu_y = []; mat_Sigma_y = [];
                vec_mu_z = []; mat_Sigma_z = [];
                vec_cost = []; mat_Jacobian = [];
                
                % for each constraint: two instants, one mark id
                for k = 1:numel(cell_cnstr)
                    
                    % read
                    struct_cnstr = cell_cnstr{k};
                    
                    % measurement
                    [vec_mu_y, mat_Sigma_y] = solver.Measure(struct_cnstr);
                    
                    % compute Hessian, Jacobian and Cost
                    switch flag
                        case 0
                            [vec_cost, mat_Jacobian, cellmat_Hessian] = solver.CostJacobHess_Spatio(vec_mu_x, vec_mu_y);
                        case 2
                            [vec_cost, mat_Jacobian, cellmat_Hessian] = solver.CostJacobHess_SpatioOdo(vec_mu_x, vec_mu_y);
                    end
                    
                    % virtual measurement
                    [vec_mu_z, mat_Sigma_z] = solver.VirtualMeasure(mat_Sigma_x, mat_Sigma_y, cellmat_Hessian);
                    
                    % correct
                    [vec_mu_x_new, mat_Sigma_x_new] = solver.Correct(...
                        vec_mu_x, mat_Sigma_x, vec_mu_y, mat_Sigma_y, ...
                        vec_mu_z, mat_Sigma_z, vec_cost, mat_Jacobian);
                    
                    % refine
                    mat_Sigma_x_new = solver.Refine(flag, vec_mu_x, vec_mu_x_new, mat_Sigma_x_new);
                    
                    % set
                    solver.vec_mu_x = vec_mu_x_new;
                    solver.mat_Sigma_x = mat_Sigma_x_new;
                    
                end
            end
        end
        
        
        % record
        rec_mu_x = [rec_mu_x; solver.vec_mu_x.'];
        rec_cov_x{end+1, 1} = solver.mat_Sigma_x;
        
        qvec_b_c = solver.vec_mu_x(1:4);
        rvec_b_c = rodrigues(quat2rot(qvec_b_c));
        tvec_b_c = [solver.vec_mu_x(5:6);0];
        
        drvec_b_c = drvec(rvec_b_c, rvec_b_c_true);
        dtvec_b_c = tvec_b_c - tvec_b_c_true;
        
        rec_rvec = [rec_rvec; rvec_b_c.'];
        rec_tvec = [rec_tvec; tvec_b_c.'];
        rec_drvec = [rec_drvec; drvec_b_c.'];
        rec_dtvec = [rec_dtvec; dtvec_b_c.'];
        
        % refresh sliding window
        solver.struct_slidingWindow = solver.RenewSlidingWindow(lp_now, measure, solver.struct_slidingWindow);
        
    end
    
    %% save
    NameRec = ['temp/Rec-s', num2str(SetId), '.mat'];
    save(NameRec, 'rec_rvec', 'rec_tvec', 'rec_drvec', 'rec_dtvec',...
        'vec_lp', 'rvec_b_c_true', 'tvec_b_c_true');
    
    %% show
    
    switch flag
        
        case 0
            vec_id = [1;2;3;4];
            DrawPlotEnv( rec_mu_x, rec_cov_x, vec_id );
            vec_id = [5;6];
            DrawPlotEnv( rec_mu_x, rec_cov_x, vec_id );
            
        case 2
            vec_id = [1;2;3];
            DrawPlotEnv( rec_mu_x, rec_cov_x, vec_id );
            vec_id = [4;5];
            DrawPlotEnv( rec_mu_x, rec_cov_x, vec_id );
            vec_id = [6;7];
            DrawPlotEnv( rec_mu_x, rec_cov_x, vec_id );
            
        otherwise
            
    end
    
    disp(tvec_b_c);
    disp(rvec_b_c);
    
end






