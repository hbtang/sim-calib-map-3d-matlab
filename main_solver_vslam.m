%% init: create class objs and load data
% clear;
% close all;

% setting = YAML.read('setting-slam-exp-1-fast.yml');
setting = YAML.read('setting-slam-exp-1-fast.yml');

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
errConfig.stdErrRatioOdoLin = setting.error.odo.stdratio_lin;
errConfig.stdErrRatioOdoRot = setting.error.odo.stdratio_rot;
errConfig.MinStdErrOdoLin = setting.error.odo.stdmin_lin;
errConfig.MinStdErrOdoRot = setting.error.odo.stdmin_rot;
errConfig.stdErrRatioMkX = setting.error.mk.stdratio_x;
errConfig.stdErrRatioMkY = setting.error.mk.stdratio_y;
errConfig.stdErrRatioMkZ = setting.error.mk.stdratio_z;
solver = ClassSolverSlam(errConfig);

% map
map = ClassMap;

% calib
calib = ClassCalib;
rvec_b_c_init = setting.init.rvec_b_c.';
tvec_b_c_init = setting.init.tvec_b_c.';
calib.SetVecbc(rvec_b_c_init, tvec_b_c_init);

% compute velocity by interpolation
measure.odo = Odo_Interpolate(measure.odo, measure_raw.odo, measure_raw.time);
measure_raw.odo = Odo_Interpolate(measure_raw.odo, measure_raw.odo, measure_raw.time);

%% init map
map.InitMap(measure, calib);
calib.DispCalib;
% map.DrawMapWithMeasure(measure, calib, true, 'Result: init');

%% step 1: estimate ground
solver.SolveGrndPlaneLin(measure, calib);
% solver.SolveGrndPlane(measure, calib);
calib.DispCalib;

%% step 2: estimate yaw and XY
solver.SolveYawXY(measure, calib);
calib.DispCalib;

%% step 2.5: solve slam after init
map.InitMap(measure, calib);
solver.SolveSlam(measure, calib, map);
[ err_Mk_1, err_Odo_1, err_MkNorm_1, err_OdoNorm_1 ] = Err_Slam( measure, calib, map, true, solver.errConfig );
calib.DispCalib;

options_drawmap.bDrawMeasure = true;
options_drawmap.strTitle = 'SLAM Result: Initial';
options_drawmap.fileNameFigOut = '.\temp\slam-init';
options_drawmap.bDrawMkRot = true;
options_drawmap.scaleMk = 3;
map.DrawMap(measure, calib, setting, options_drawmap);

%% step 3: spatio joint optimization
solver.SolveJointOpt2(measure, calib, map);
[ err_Mk_2, err_Odo_2, err_MkNorm_2, err_OdoNorm_2 ] = Err_Slam( measure, calib, map, true, solver.errConfig );
calib.DispCalib;

flag_initmap.bKfsOdo = false;
flag_initmap.bMksLin = false;
flag_initmap.bMksRot = true;
map.InitMap(measure, calib, flag_initmap);

options_drawmap.bDrawMeasure = true;
options_drawmap.strTitle = 'SLAM Result: Spatio';
options_drawmap.fileNameFigOut = '.\temp\slam-spatio';
options_drawmap.bDrawMkRot = true;
options_drawmap.scaleMk = 3;
map.DrawMap(measure, calib, setting, options_drawmap);

%% step 3: spatio-temporal joint optimization
% solver.SolveJointOpt3(measure, calib, map);
% [ err_Mk_3, err_Odo_3, err_MkNorm_3, err_OdoNorm_3 ] = Err_Slam( measure, calib, map, true, solver.errConfig );
% calib.DispCalib;
% fileNameFigOut = '.\temp\slam-spatio-temporal';
% map.DrawMapWithMeasure(measure, calib, true, 'SLAM Result: Spatio-Temporal Calib.', fileNameFigOut);

%% step 3: spatio-temporal-odometric joint optimization
% solver.SolveJointOpt4(measure, calib, map);
% [ err_Mk_4, err_Odo_4, err_MkNorm_4, err_OdoNorm_4 ] = Err_Slam( measure, calib, map, true, solver.errConfig );
% calib.DispCalib;
% fileNameFigOut = '.\temp\slam-spatio-temporal-odo';
% map.DrawMapWithMeasure(measure, calib, true, 'SLAM Result: Spatio-Temporal-Odometric Calib.', fileNameFigOut);

%% step 3: spatio-temporal-odometric joint optimization with visual measurements
flag_vslam.bCalibExt = true;
flag_vslam.bCalibTmp = false;
flag_vslam.bCalibOdo = false;
solver.SolveJointOpt5(measure, calib, map, setting, flag_vslam);
calib.DispCalib;

options_drawmap.strTitle = 'V-SLAM Result: Spatio';
options_drawmap.fileNameFigOut = '.\temp\vslam-spatio';
options_drawmap.bDrawMeasure = true;
options_drawmap.bDrawMkRot = true;
options_drawmap.scaleMk = 3;
map.DrawMap(measure, calib, setting, options_drawmap);

%% Draw Error
% cellMat_errMk = {err_Mk_1, err_Mk_2, err_Mk_3, err_Mk_4};
% cellMat_errOdo = {err_Odo_1, err_Odo_2, err_Odo_3, err_Odo_4};
% cellMat_mkNorm = {err_MkNorm_1, err_MkNorm_2, err_MkNorm_3, err_MkNorm_4};
% cellMat_odoNorm = {err_OdoNorm_1, err_OdoNorm_2, err_OdoNorm_3, err_OdoNorm_4};
% DrawErr_Slam(cellMat_errMk, cellMat_errOdo, cellMat_mkNorm, cellMat_odoNorm);
%
% disp('Root-Mean-Square Error: Init. Results');
% PrintRmsErr( err_Mk_1, err_Odo_1, err_MkNorm_1, err_OdoNorm_1 );
% disp(' ');
% disp('Root-Mean-Square Error: Spatio Joint Opt. Results');
% PrintRmsErr( err_Mk_2, err_Odo_2, err_MkNorm_2, err_OdoNorm_2 );
% disp(' ');
% disp('Root-Mean-Square Error: Spatio-Temporal Joint Opt. Results');
% PrintRmsErr( err_Mk_3, err_Odo_3, err_MkNorm_3, err_OdoNorm_3 );
% disp(' ');
% disp('Root-Mean-Square Error: Spatio-Temporal-Odo Joint Opt. Results');
% PrintRmsErr( err_Mk_4, err_Odo_4, err_MkNorm_4, err_OdoNorm_4 );
% disp(' ');

%% Debug








