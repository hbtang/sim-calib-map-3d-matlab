%% init: create class objs and load data
% clear;
% close all;

setting = YAML.read('setting-slam-sim-1.yml');
% setting = YAML.read('setting-slam-exp-1-fast.yml');

% measure
measure = ClassMeasure(setting.path.fold, setting.path.markfilename, setting.path.odofilename);
measure.ReadRecData;

%%%%%%%% Debug %%%%%%%%
% AGV odo time data is not correct!!!
% measure.time.t_odo = TimeOdo_ConstVel(measure.time.t_odo);
% measure.time.t_odo = TimeOdo_EqualOffset(measure.time.t_odo, measure.odo);
measure.time.t_odo = measure.time.t_mk;
%%%%%%%% Debug %%%%%%%%

measure_raw = ClassMeasure();
measure.CopyTo(measure_raw);
measure.PruneData(setting.prune.thresh_lin, setting.prune.thresh_rot);

% solver
solver = ClassSolverSlam(setting.error);

% map
map = ClassMap;

% calib
calib = ClassCalib;
rvec_b_c_init = setting.init.rvec_b_c.';
tvec_b_c_init = setting.init.tvec_b_c.';
calib.SetVecbc(rvec_b_c_init, tvec_b_c_init);
calib.mat_camera = setting.camera.camera_matrix;
calib.vec_distortion = setting.camera.distortion_coefficients;

% compute velocity by interpolation
measure.odo = Odo_Interpolate(measure.odo, measure_raw.odo, measure_raw.time);
measure_raw.odo = Odo_Interpolate(measure_raw.odo, measure_raw.odo, measure_raw.time);

%% init map
map.InitMap(measure, calib);
% calib.DispCalib;

%% step 1.1: init. estimate ground
solver.SolveGrndPlaneLin(measure, calib);
% calib.DispCalib;
%% step 1.2: init. estimate yaw and XY
solver.SolveYawXY(measure, calib);
calib.DispCalib;

%% debug
solver.SolveInitGuo(measure, calib);
calib.DispCalib;

%% step 1.3: v-slam after init.
% init. map from odometry and mark observation
map.InitMap(measure, calib);
% do vslam with current calibration results
options_mslam = struct(...
    'bCalibExtRot', false, 'bCalibExtLin', false,...
    'bCalibTmp', false, 'bCalibOdo', false);
solver.SolveJointOptVSlam(measure, calib, map, setting, options_mslam);
% display init-calib results
calib.DispCalib;
% draw v-slam results
options_drawmap = struct('strTitle', 'v-SLAM Result: Init. Calib.', 'fileNameFigOut', '.\temp\mslam-init', ...
    'bDrawMeasure', true, 'bDrawMkRot', true, 'scaleMk', 3);
map.DrawMap(measure, calib, setting, options_drawmap);
% compute vslam error
options_errvslam = struct('bCalibTmp', true, 'bCalibOdo', true);
struct_errvslam_init = Err_vSlam( measure, calib, map, setting, options_errvslam );

%% step 2: calib with m-slam
% do m-slam-calib
options_mslam = struct(...
    'bCalibExtRot', true, 'bCalibExtLin', true,...
    'bCalibTmp', true, 'bCalibOdo', true);
solver.SolveJointOptMSlam(measure, calib, map, setting, options_mslam);
% display m-slam-calib results
calib.DispCalib;
% do v-slam based on m-slam-calib results
options_mslam = struct(...
    'bCalibExtRot', false, 'bCalibExtLin', false,...
    'bCalibTmp', false, 'bCalibOdo', false);
solver.SolveJointOptVSlam(measure, calib, map, setting, options_mslam);
% draw v-slam results
options_drawmap = struct('strTitle', 'v-SLAM Result: m-Calib.', 'fileNameFigOut', '.\temp\mslam-all', ...
    'bDrawMeasure', true, 'bDrawMkRot', true, 'scaleMk', 3);
map.DrawMap(measure, calib, setting, options_drawmap);
% compute vslam error 
options_errvslam = struct('bCalibTmp', true, 'bCalibOdo', true);
struct_errvslam_mcalib = Err_vSlam( measure, calib, map, setting, options_errvslam );

%% step 3: calib with v-slam
% do v-slam-calib
options_vslam = struct(...
    'bCalibExtRot', true, 'bCalibExtLin', true,...
    'bCalibTmp', true, 'bCalibOdo', true, ...
    'bCalibCamMat', true, 'bCalibCamDist', true);
solver.SolveJointOptVSlam(measure, calib, map, setting, options_vslam);
% display v-slam-calib results
calib.DispCalib;
% draw v-slam results
options_drawmap = struct('strTitle', 'v-SLAM Result: v-Calib.', 'fileNameFigOut', '.\temp\vslam-all', ...
    'bDrawMeasure', true, 'bDrawMkRot', true, 'scaleMk', 3);
map.DrawMap(measure, calib, setting, options_drawmap);
% compute v-slam error
options_errvslam = struct('bCalibTmp', true, 'bCalibOdo', true);
struct_errvslam_vcalib = Err_vSlam( measure, calib, map, setting, options_errvslam );

%% Draw Error
% to be blocked...
figure; grid on; axis equal; hold on;
plot(struct_errvslam_init.mat_errImg(:,1), struct_errvslam_init.mat_errImg(:,2), '.', 'Color', 'b');
plot(struct_errvslam_init.mat_errImg(:,3), struct_errvslam_init.mat_errImg(:,4), '.', 'Color', 'g');
plot(struct_errvslam_init.mat_errImg(:,5), struct_errvslam_init.mat_errImg(:,6), '.', 'Color', 'r');
plot(struct_errvslam_init.mat_errImg(:,7), struct_errvslam_init.mat_errImg(:,8), '.', 'Color', 'c');

figure; grid on; axis equal; hold on;
plot(struct_errvslam_mcalib.mat_errImg(:,1), struct_errvslam_mcalib.mat_errImg(:,2), '.', 'Color', 'b');
plot(struct_errvslam_mcalib.mat_errImg(:,3), struct_errvslam_mcalib.mat_errImg(:,4), '.', 'Color', 'g');
plot(struct_errvslam_mcalib.mat_errImg(:,5), struct_errvslam_mcalib.mat_errImg(:,6), '.', 'Color', 'r');
plot(struct_errvslam_mcalib.mat_errImg(:,7), struct_errvslam_mcalib.mat_errImg(:,8), '.', 'Color', 'c');

figure; grid on; axis equal; hold on;
plot(struct_errvslam_vcalib.mat_errImg(:,1), struct_errvslam_vcalib.mat_errImg(:,2), '.', 'Color', 'b');
plot(struct_errvslam_vcalib.mat_errImg(:,3), struct_errvslam_vcalib.mat_errImg(:,4), '.', 'Color', 'g');
plot(struct_errvslam_vcalib.mat_errImg(:,5), struct_errvslam_vcalib.mat_errImg(:,6), '.', 'Color', 'r');
plot(struct_errvslam_vcalib.mat_errImg(:,7), struct_errvslam_vcalib.mat_errImg(:,8), '.', 'Color', 'c');

%% Debug








