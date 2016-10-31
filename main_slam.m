%% init: create class objs and load data
% clear;

[FileName,PathName] = uigetfile('*.xml', 'Select the setting file', 'C:\Workspace\Data\');
settingFile = [PathName, FileName];
% settingFile = 'C:\Workspace\Data\sim\sim-sqrmap-inout-2016.1.18\config\setting-simcalibmap3d-slam.xml';
% settingFile = 'C:\Workspace\Data\cuhksz-2016.3.15\r2-rightback-bidir-mk127-2016031520\config\setting-simcalibmap3d-slam.xml';

configXml = readXml_slam(settingFile);

% measure
measure = ClassMeasure(configXml.PathFold, configXml.NameMk, configXml.NameOdo);
measure.ReadRecData;
measure.PruneData(configXml.ThreshTransPruneData, configXml.ThreshRotPruneData);

% solver
errConfig.stdErrRatioOdoLin = configXml.SolverConfig_StdErrRatioOdoLin;
errConfig.stdErrRatioOdoRot = configXml.SolverConfig_StdErrRatioOdoRot;
errConfig.stdErrRatioMkX = configXml.SolverConfig_StdErrRatioMkX;
errConfig.stdErrRatioMkY = configXml.SolverConfig_StdErrRatioMkY;
errConfig.stdErrRatioMkZ = configXml.SolverConfig_StdErrRatioMkZ;
solver = ClassSolver(errConfig);

% map
map = ClassMap;

% calib
calib = ClassCalib;
qcbInit = configXml.qcbInit;
pt3cbInit = configXml.pt3cbInit;
RcbInit = quat2rot(qcbInit);
rveccbInit = rodrigues(RcbInit);
calib.SetVecbc(rveccbInit, pt3cbInit);

%% init map
map.InitMap(measure, calib);
map.DrawMapWithMeasure(measure, calib, true, 'Result: init');
calib.DispCalib;

%% solve step 1: estimate ground
solver.SolveGrndPlaneLin(measure, calib);
% solver.SolveGrndPlane(measure, calib);
map.InitMap(measure, calib);
map.DrawMapWithMeasure(measure, calib, true, 'Result: with ground estimated');
calib.DispCalib;

%% solve step 2: estimate yaw and XY
solver.SolveYawXY(measure, calib);
map.InitMap(measure, calib);
map.DrawMapWithMeasure(measure, calib, true, 'Result: with yaw and translation estimated');
calib.DispCalib;

%% solve step 2: local optimization
% solver.SolveLocalOpt(measure, calib);
% map.InitMap(measure, calib);
% map.DrawMapWithMeasure(measure, calib);
% calib.DispPs2dbcg;

%% solve step 2: by local opt with loop closing
% solver.SolveLocalLoopOpt(measure, calib);
% map.InitMap(measure, calib);
% map.DrawMapWithMeasure(measure, calib);
% calib.DispPs2dbcg;

%% solve step 3: joint optimization
% solver.SolveJointOpt(measure, calib, map);
solver.SolveJointOpt2(measure, calib, map);
calib.DispCalib;

%% draw final SLAM results
strTitle = 'Result: after joint optimization';
map.DrawMapWithMeasure(measure, calib, true, strTitle);
% save figure ...
set(gcf, 'PaperPositionMode', 'auto');
fileNameFigOutput = '.\temp\slam';
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');




