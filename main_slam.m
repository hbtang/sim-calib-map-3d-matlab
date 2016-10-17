%% init: create class objs and load data
clear;

settingFile = 'C:\Workspace\Data\sim\sim-sqrmap-inout-2016.1.18\config\setting-simcalibmap3d-slam.xml';
configXml = readXml_slam(settingFile);

% measure
PathFold = configXml.PathFold;
NameMk = configXml.NameMk;
NameOdo = configXml.NameOdo;

measure = ClassMeasure(PathFold, NameMk, NameOdo);
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
map.DrawMapWithMeasure(measure, calib);
calib.DispPs2dbcg;

%% solve step 1: estimate ground
solver.SolveGrndPlane(measure, calib);
map.InitMap(measure, calib);
map.DrawMapWithMeasure(measure, calib);
calib.DispPs2dbcg;

%% solve step 2: local optimization
solver.SolveLocalOpt(measure, calib);
map.InitMap(measure, calib);
map.DrawMapWithMeasure(measure, calib);
calib.DispPs2dbcg;

%% solve step 2: by local opt with loop closing
solver.SolveLocalLoopOpt(measure, calib);
map.InitMap(measure, calib);
map.DrawMapWithMeasure(measure, calib);
calib.DispPs2dbcg;

%% solve step 3: joint optimization
solver.SolveJointOpt(measure, calib, map);
calib.DispPs2dbcg;

%% draw final SLAM results
map.DrawMapWithMeasure(measure, calib);
set(gcf, 'Position', [1,1,480,360]);
ax = gca;
ax.XTick = (-10000:2000:10000);
ax.YTick = (-10000:2000:10000);
ax.XLim = [-10000 10000];
ax.YLim = [-9000 8000];
strTitle = 'Joint SLAM & Calibration: Simulation Dataset';
title(strTitle, 'FontWeight','bold');
xlabel('X (mm)');
ylabel('Y (mm)');
box on;
set(gcf, 'PaperPositionMode', 'auto');
fileNameFigOutput = '.\temp\slam';
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');




