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
tvec_b_c_init = setting.init.tvec_b_c;
rvec_b_c_init = setting.init.rvec_b_c;
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
tvec_b_c_true = setting.truth.tvec_b_c;
rvec_b_c_true = setting.truth.rvec_b_c;

% init solver
solver = ClassSolverEkf(errConfig);
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
    
    %% init mk if mk is new
    [struct_measure] = solver.InitMkNew(struct_measure);
    
    %% propagate
    
    
    %% correct
    
    
    %% record
     
end







