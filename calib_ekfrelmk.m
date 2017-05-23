% main_vmckf_sw: run vmckf with sliding-window

%% init: create class objs and load data
clear;

% read configure file
setting = YAML.read('setting-gf-sim.yml');
% setting = YAML.read('setting-gf-exp.yml');

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


%% init solver
% init calib
calib = ClassCalib;
tvec_b_c_init = setting.init.tvec_b_c.';
rvec_b_c_init = setting.init.rvec_b_c.';
calib.SetVecbc(rvec_b_c_init, tvec_b_c_init);
calib.dt = 0;
calib.k_odo_lin = 1;
calib.k_odo_rot = 1;
% create solver
solver = ClassSolverEkfRelMk(err_config);
solver.SetStateFromCalib(calib,0);
solver.InitBasePose(measure);

%% solve in rt, main loop
vec_lp = measure.odo.lp;
rec_mu_x = [];
rec_cov_x = cell(0,0);
rec_err_rvec = [];
rec_err_xy = [];

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
    R_b_c_now = rodrigues(solver.vec_mu_x(1:3));
    err_rvec = rodrigues(R_b_c_now * R_b_c_true.');
    
    rec_err_rvec = [rec_err_rvec; err_rvec.'];
    rec_err_xy = [rec_err_xy; solver.vec_mu_x(4:5).' - tvec_b_c_true(1:2).'];
    
    rec_mu_x = [rec_mu_x; solver.vec_mu_x(1:6).' - vec_mu_x_true.'];
    rec_cov_x{end+1, 1} = solver.mat_Sigma_x(1:6,1:6);
    
    %% debug
    %     solver.vec_mkid.'
    %     disp(i);
    
end

%% show
% figure; hold on; grid on;
% plot(rec_err_rvec);
% figure; hold on; grid on;
% plot(rec_err_xy);

vec_id = [1;2;3];
DrawPlotEnv( rec_mu_x, rec_cov_x, vec_id );
vec_id = [4;5];
DrawPlotEnv( rec_mu_x, rec_cov_x, vec_id );
% vec_id = [7;8];
% DrawPlotEnv( rec_mu_x, rec_cov_x, vec_id );





