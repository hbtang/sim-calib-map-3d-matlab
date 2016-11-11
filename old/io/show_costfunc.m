%% init: create class objs and load data
clear;

% measure
PathFold = './data/sim_20160119_ptmap_zzstraight';
NameMk = 'Mk.rec'; NameOdo = 'OdoNs-0.01-0.002.rec';
measure = ClassMeasure(PathFold, NameMk, NameOdo);
measure.ReadRecData;
measure.PruneData(200, 5*pi/180);

% solver
errConfig.stdErrRatioOdoLin = 0.03;
errConfig.stdErrRatioOdoRot = 0.005;
errConfig.stdErrRatioMkX = 0.002;
errConfig.stdErrRatioMkY = 0.002;
errConfig.stdErrRatioMkZ = 0.01;
solver = ClassSolver(errConfig);

% map
map = ClassMap;

% calib
calib = ClassCalib;

%% extra display for debugging...
% draw cost value on xy
ps2d_b_cg = calib.GetPs2dbcg;

[X,Y] = meshgrid(-1000:100:1000, -1000:100:1000);
[rows, cols] = size(X);
Z = zeros(rows, cols);
for i = 1:rows
    for j = 1:cols
        q_tmp = [X(i,j);Y(i,j);ps2d_b_cg(3)];
        vecCost_tmp = solver.CostLocalOpt(q_tmp, measure, calib);
        Z(i,j) = sqrt(vecCost_tmp.'*vecCost_tmp)/numel(vecCost_tmp);
        disp([i, j]);
    end
end

figure;
surf(X,Y,Z);
% axis equal;
figure;
contour(X,Y,Z,'ShowText','on');
axis equal;

% draw on theta
vecTheta = (-pi:3*pi/180:0);
vecVal = zeros(size(vecTheta));
for i = 1:numel(vecTheta)
    q_tmp = [ps2d_b_cg(1:2);vecTheta(i)];
    vecCost_tmp = solver.CostLocalOpt(q_tmp, measure, calib);
    vecVal(i) = sqrt(vecCost_tmp.'*vecCost_tmp)/numel(vecCost_tmp);
    disp(i);
end
figure; grid on; hold on;
plot(vecTheta, vecVal);