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
MdNs = 1; OdoNs = 1; SetId = 1;

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
    measure.PruneData(setting.prune.thresh_lin, setting.prune.thresh_rot);
    
    % read error config
    err_config = setting.error;
    
    % init ground truth
    rvec_b_c_true = setting.truth.rvec_b_c.';
    tvec_b_c_true = setting.truth.tvec_b_c.';
    vec_mu_x_true = [rvec_b_c_true; tvec_b_c_true];
    R_b_c_true = rodrigues(rvec_b_c_true);
    
    % init calib
    calib = ClassCalib;
    rvec_b_c_init = setting.init.rvec_b_c.';
    tvec_b_c_init = setting.init.tvec_b_c.';
%     rvec_b_c_init = normrnd([rvec_b_c_init], [0.3; 0.3; 0.3]);
%     tvec_b_c_init = normrnd([tvec_b_c_init], [500; 500; 0]);
    calib.SetVecbc(rvec_b_c_init, tvec_b_c_init);
    
    %% init solver
    % create solver
    solver = ClassSolverEkfRelMk(err_config);
    solver.SetStateFromCalib(calib,0);
    solver.InitBasePose(measure);
    
    %% solve in rt, main loop
    vec_lp = measure.odo.lp;
    
    % create record;
    rec_mu_x = [];
    rec_cov_x = cell(0,0);
    
    rec_tvec = [];
    rec_rvec = [];
    rec_dtvec = [];
    rec_drvec = [];
    rec_lp = [];
    
    for i = 1:(numel(vec_lp))
        %% read measure once
        [b_read_fail, struct_measure] = solver.ReadMeasure(measure);
        if b_read_fail
            break;
        end
        
        %% propagate
        solver.Propagate(struct_measure);
        
        %% init mk if mk is new
        [struct_measure] = solver.InitMkNew(struct_measure);
        
        %% correct
        solver.Correct(struct_measure);
        
        %% record        
        rec_mu_x = [rec_mu_x; solver.vec_mu_x(1:6).' - vec_mu_x_true.'];
        rec_cov_x{end+1, 1} = solver.mat_Sigma_x(1:6,1:6);
        
        rvec_b_c = solver.GetRvec;
        tvec_b_c = solver.GetTvec;
        drvec_b_c = drvec(rvec_b_c, rvec_b_c_true);
        dtvec_b_c = tvec_b_c - tvec_b_c_true;
        
        rec_rvec = [rec_rvec; rvec_b_c.'];
        rec_tvec = [rec_tvec; tvec_b_c.'];
        rec_drvec = [rec_drvec; drvec_b_c.'];        
        rec_dtvec = [rec_dtvec; dtvec_b_c.'];
        rec_lp = [rec_lp; vec_lp(i)];
        
    end
    
    %% save
    NameRec = ['temp/Rec-s', num2str(SetId), '.mat'];
    vec_lp = rec_lp;
    save(NameRec, 'rec_rvec', 'rec_tvec', 'rec_drvec', 'rec_dtvec', ...
        'vec_lp', 'rvec_b_c_true', 'tvec_b_c_true');
    
    %% show
%     vec_id = [1;2;3];
%     DrawPlotEnv( rec_mu_x, rec_cov_x, vec_id );
%     vec_id = [4;5];
%     DrawPlotEnv( rec_mu_x, rec_cov_x, vec_id );
    plot(rec_drvec);
    plot(rec_dtvec);
    
    disp(tvec_b_c);
    disp(rvec_b_c);
    
end



