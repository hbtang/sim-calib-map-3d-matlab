%% init: create class objs and load data
clear;

% measure
PathFold = '../data/r1-right-jx1-mk127-20160627';
% PathFold = '../data/sim-sqrmap-inout-20160118';
NameMk = 'Mk.rec'; NameOdo = 'Odo.rec';
measure = ClassMeasure(PathFold, NameMk, NameOdo);
measure.ReadRecData;
measure.PruneData(200, 10*pi/180);

% solver
errConfig.stdErrRatioOdoLin = 0.03;
errConfig.stdErrRatioOdoRot = 0.005;
errConfig.stdErrRatioMkX = 0.002;
errConfig.stdErrRatioMkY = 0.002;
errConfig.stdErrRatioMkZ = 0.01;

% errConfig.stdErrRatioOdoLin = 0.05;
% errConfig.stdErrRatioOdoRot = 0.1;
% errConfig.stdErrRatioMkX = 0.01;
% errConfig.stdErrRatioMkY = 0.01;
% errConfig.stdErrRatioMkZ = 0.05;
solver = ClassSolver(errConfig);

% map
map = ClassMap;

% calib
calib = ClassCalib;
% calib.SetPs2dbcg([-1000;1000;-pi/4]);

%% debuging: refine map
% measure.odo = FunRefineOdo(measure.odo, 1, 0.97);

% for rec_p1_right_clock_mk120_2016031509
% measure.odo = FunRefineOdo(measure.odo, 1, 0.945);

% for rec_p1_right_anticlock_mk120_2016031422
% measure.odo = FunRefineOdo(measure.odo, 1, 0.97);

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




