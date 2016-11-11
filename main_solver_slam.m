%% init: create class objs and load data
% clear;

% [FileName,PathName] = uigetfile('*.xml', 'Select the setting file', 'C:\Workspace\Data\');
% settingFile = [PathName, FileName];
% settingFile = 'C:\Workspace\Data\sim\sim-sqrmap-inout-2016.1.18\config\setting-simcalibmap3d-slam.xml';
% settingFile = 'C:\Workspace\Data\cuhksz-2016.3.15\r2-rightback-bidir-mk127-2016031520\config\setting-simcalibmap3d-slam.xml';
settingFile = 'setting-slam-exp-1.xml';
configXml = readXml_slam(settingFile);

% measure
measure = ClassMeasure(configXml.PathFold, configXml.NameMk, configXml.NameOdo);
measure.ReadRecData;

%%%%%%%% Debug %%%%%%%%
% AGV odo time data is not correct!!!
% measure.time.t_odo = TimeOdo_ConstVel(measure.time.t_odo);
% measure.time.t_odo = TimeOdo_EqualOffset(measure.time.t_odo, measure.odo);
measure.time.t_odo = measure.time.t_mk;
%%%%%%%% Debug %%%%%%%%

measure_raw = ClassMeasure();
measure.CopyTo(measure_raw);
measure.PruneData(configXml.ThreshTransPruneData, configXml.ThreshRotPruneData);

% solver
errConfig.stdErrRatioOdoLin = configXml.SolverConfig_StdErrRatioOdoLin;
errConfig.stdErrRatioOdoRot = configXml.SolverConfig_StdErrRatioOdoRot;
errConfig.MinStdErrOdoLin = configXml.SolverConfig_MinStdErrOdoLin;
errConfig.MinStdErrOdoRot = configXml.SolverConfig_MinStdErrOdoRot;
errConfig.stdErrRatioMkX = configXml.SolverConfig_StdErrRatioMkX;
errConfig.stdErrRatioMkY = configXml.SolverConfig_StdErrRatioMkY;
errConfig.stdErrRatioMkZ = configXml.SolverConfig_StdErrRatioMkZ;
solver = ClassSolverSlam(errConfig);

% map
map = ClassMap;

% calib
calib = ClassCalib;
qcbInit = configXml.qcbInit;
pt3cbInit = configXml.pt3cbInit;
RcbInit = quat2rot(qcbInit);
rveccbInit = rodrigues(RcbInit);
calib.SetVecbc(rveccbInit, pt3cbInit);

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
% map.InitMap(measure, calib);
% map.DrawMapWithMeasure(measure, calib, true, 'Result: ground estimated');

%% step 2: estimate yaw and XY
solver.SolveYawXY(measure, calib);
calib.DispCalib;
% map.InitMap(measure, calib);
% map.DrawMapWithMeasure(measure, calib, true, 'Result: yaw and translation estimated');


%% step 2.5: solve slam after init
solver.SolveSlam(measure, calib, map);
[ err_Mk_1, err_Odo_1, err_MkNorm_1, err_OdoNorm_1 ] = Err_Slam( measure, calib, map, true, solver.errConfig );
calib.DispCalib;
fileNameFigOut = '.\temp\slam-init';
map.DrawMapWithMeasure(measure, calib, true, 'Result: with init calib', fileNameFigOut);

%% step 3: spatio joint optimization
solver.SolveJointOpt2(measure, calib, map);
[ err_Mk_2, err_Odo_2, err_MkNorm_2, err_OdoNorm_2 ] = Err_Slam( measure, calib, map, true, solver.errConfig );
calib.DispCalib;
fileNameFigOut = '.\temp\slam-jointopt';
map.DrawMapWithMeasure(measure, calib, true, 'Result: joint opt. spatio calib.', fileNameFigOut);

%% step 3: spatio-temporal joint optimization
solver.SolveJointOpt3(measure, calib, map);
[ err_Mk_3, err_Odo_3, err_MkNorm_3, err_OdoNorm_3 ] = Err_Slam( measure, calib, map, true, solver.errConfig );
calib.DispCalib;
fileNameFigOut = '.\temp\slam-jointopttime';
map.DrawMapWithMeasure(measure, calib, true, 'Result: joint opt. temporal-spatio calib.', fileNameFigOut);

%% step 3: spatio-temporal-odometric joint optimization
solver.SolveJointOpt4(measure, calib, map);
[ err_Mk_4, err_Odo_4, err_MkNorm_4, err_OdoNorm_4 ] = Err_Slam( measure, calib, map, true, solver.errConfig );
calib.DispCalib;
fileNameFigOut = '.\temp\slam-jointopttimeodo';
map.DrawMapWithMeasure(measure, calib, true, 'Result: joint opt. with temporal-spatio-odo calib.', fileNameFigOut);

%% Draw Error
cellMat_errMk = {err_Mk_1, err_Mk_2, err_Mk_3, err_Mk_4};
cellMat_errOdo = {err_Odo_1, err_Odo_2, err_Odo_3, err_Odo_4};
cellMat_mkNorm = {err_MkNorm_1, err_MkNorm_2, err_MkNorm_3, err_MkNorm_4};
cellMat_odoNorm = {err_OdoNorm_1, err_OdoNorm_2, err_OdoNorm_3, err_OdoNorm_4};
DrawErr_Slam(cellMat_errMk, cellMat_errOdo, cellMat_mkNorm, cellMat_odoNorm);

disp('Root-Mean-Square Error: Init. Results');
PrintRmsErr( err_Mk_1, err_Odo_1, err_MkNorm_1, err_OdoNorm_1 );
disp(' ');
disp('Root-Mean-Square Error: Spatio Joint Opt. Results');
PrintRmsErr( err_Mk_2, err_Odo_2, err_MkNorm_2, err_OdoNorm_2 );
disp(' ');
disp('Root-Mean-Square Error: Spatio-Temporal Joint Opt. Results');
PrintRmsErr( err_Mk_3, err_Odo_3, err_MkNorm_3, err_OdoNorm_3 );
disp(' ');
disp('Root-Mean-Square Error: Spatio-Temporal-Odo Joint Opt. Results');
PrintRmsErr( err_Mk_4, err_Odo_4, err_MkNorm_4, err_OdoNorm_4 );
disp(' ');


