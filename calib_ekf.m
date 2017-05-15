% main_vmckf_sw: run vmckf with sliding-window

%% init: create class objs and load data
clear;

% read configure file
setting = YAML.read('setting-vmckf-sim.yml');

% read record data
measure = ClassMeasure(setting.path.fold, setting.path.markfilename, setting.path.odofilename);
measure.ReadRecData;
measure.PruneData(setting.prune.thresh_lin, setting.prune.thresh_rot);

% init calib
calib = ClassCalib;
tvec_b_c_init = setting.init.tvec_b_c.';
rvec_b_c_init = setting.init.rvec_b_c.';
calib.SetVecbc(rvec_b_c_init, tvec_b_c_init);
calib.dt = 0;
calib.k_odo_lin = 1;
calib.k_odo_rot = 1;

% read error config
err_config = setting.error;

% init ground truth
rvec_b_c_true = setting.truth.rvec_b_c.';
tvec_b_c_true = setting.truth.tvec_b_c.';
vec_mu_x_true = [rvec_b_c_true; tvec_b_c_true];

% init solver
solver = ClassSolverEkf(err_config);
solver.SetStateFromCalib(calib);
solver.InitBasePose(measure);

%% solve in rt, main loop
vec_lp = measure.odo.lp;
rec_mu_x = [];
rec_cov_x = cell(0,0);

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
    
end

%% show
vec_id = [1;2;3];
DrawPlotEnv( rec_mu_x, rec_cov_x, vec_id );
vec_id = [4;5];
DrawPlotEnv( rec_mu_x, rec_cov_x, vec_id );





