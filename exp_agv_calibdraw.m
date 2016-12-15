% do calib and record results, by simulation dataset: initguo, init, mslam, vslam
clear;
close all;

bCalibSp = true;
bCalibSpOdo = true;
bCalibSpOdoTmp = true;

%% init: create class objs and load data
% setting
setting = YAML.read('setting-slam-exp-1-fast.yml');

% measure
measure = ClassMeasure(setting.path.fold, setting.path.markfilename, setting.path.odofilename);
measure.ReadRecData;

%%%%%%%% Debug %%%%%%%%
% AGV odo time data is not correct!!!
measure.time.t_odo = measure.time.t_mk;
%%%%%%%% Debug %%%%%%%%

measure_raw = ClassMeasure();
measure.CopyTo(measure_raw);
measure.PruneData(setting.prune.thresh_lin, setting.prune.thresh_rot);

% compute velocity by interpolation
measure.odo = Odo_Interpolate(measure.odo, measure_raw.odo, measure_raw.time, 1);

% solver
solver = ClassSolverSlam(setting);

% map
map = ClassMap;

% calib
calib = ClassCalib(setting);

% slam configure
options_mslam = struct(...
    'bCalibExtRot', false, 'bCalibExtLin', false,...
    'bCalibTmp', false, 'bCalibOdo', false);
options_mslam_sp = struct(...
    'bCalibExtRot', true, 'bCalibExtLin', true,...
    'bCalibTmp', false, 'bCalibOdo', false);
options_mslam_spodo = struct(...
    'bCalibExtRot', true, 'bCalibExtLin', true,...
    'bCalibTmp', false, 'bCalibOdo', true);
options_mslam_spodotemp = struct(...
    'bCalibExtRot', true, 'bCalibExtLin', true,...
    'bCalibTmp', true, 'bCalibOdo', true);
options_vslam = struct(...
    'bCalibExtRot', false, 'bCalibExtLin', false,...
    'bCalibTmp', false, 'bCalibOdo', false, ...
    'bCalibCamMat', false, 'bCalibCamDist', false);
options_vslam_sp = struct(...
    'bCalibExtRot', true, 'bCalibExtLin', true,...
    'bCalibTmp', false, 'bCalibOdo', false, ...
    'bCalibCamMat', false, 'bCalibCamDist', false);
options_vslam_spodo = struct(...
    'bCalibExtRot', true, 'bCalibExtLin', true,...
    'bCalibTmp', false, 'bCalibOdo', true, ...
    'bCalibCamMat', false, 'bCalibCamDist', false);
options_vslam_spodotemp = struct(...
    'bCalibExtRot', true, 'bCalibExtLin', true,...
    'bCalibTmp', true, 'bCalibOdo', true, ...
    'bCalibCamMat', false, 'bCalibCamDist', false);
options_errvslam = struct(...
    'bCalibTmp', true, ...
    'bCalibOdo', true);
%% init by guo's solution
solver.SolveInitGuo(measure, calib);
calib.DispCalib;
struct_results.initvo = calib.GetCalibResult;
% do v-slam and compute error
map.InitMap(measure, calib);
solver.SolveJointOptVSlam(measure, calib, map, options_vslam);
struct_errors.initvo = Err_vSlam( measure, calib, map, setting, options_errvslam );

%% init by my solution
solver.SolveGrndPlaneLin(measure, calib);
solver.SolveYawXY(measure, calib);
calib.DispCalib;
struct_results.initmk = calib.GetCalibResult;
% do v-slam and compute error
map.InitMap(measure, calib);
solver.SolveJointOptVSlam(measure, calib, map, options_vslam);
struct_errors.initmk = Err_vSlam( measure, calib, map, setting, options_errvslam );

%% slam with sp
if bCalibSp
    %% calib with m-slam
    % init. map from odometry and mark observation
    map.InitMap(measure, calib);
    % do m-slam-calib
    solver.SolveJointOptMSlam(measure, calib, map, options_mslam_sp);
    % display v-slam-calib results
    calib.DispCalib;
    % record
    struct_results.mslam = calib.GetCalibResult;
    % do v-slam and compute error
    solver.SolveJointOptVSlam(measure, calib, map, options_vslam);
    struct_errors.mslam = Err_vSlam( measure, calib, map, setting, options_errvslam );
    
    
    %% calib with v-slam
    % do v-slam-calib
    solver.SolveJointOptVSlam(measure, calib, map, options_vslam_sp);
    % display v-slam-calib results
    calib.DispCalib;
    % record
    struct_results.vslam = calib.GetCalibResult;
    % do v-slam and compute error
    solver.SolveJointOptVSlam(measure, calib, map, options_vslam);
    struct_errors.vslam = Err_vSlam( measure, calib, map, setting, options_errvslam );
end

%% slam with sp-odo
if bCalibSpOdo
    %% calib with m-slam
    % do m-slam-calib
    solver.SolveJointOptMSlam(measure, calib, map, options_mslam_spodo);
    % display v-slam-calib results
    calib.DispCalib;
    % record
    struct_results.mslamodo = calib.GetCalibResult;
    % do v-slam and compute error
    solver.SolveJointOptVSlam(measure, calib, map, options_vslam);
    struct_errors.mslamodo = Err_vSlam( measure, calib, map, setting, options_errvslam );
    
    %% calib with v-slam
    % do v-slam-calib
    solver.SolveJointOptVSlam(measure, calib, map, options_vslam_spodo);
    % display v-slam-calib results
    calib.DispCalib;
    % record
    struct_results.vslamodo = calib.GetCalibResult;
    % do v-slam and compute error
    solver.SolveJointOptVSlam(measure, calib, map, options_vslam);
    struct_errors.vslamodo = Err_vSlam( measure, calib, map, setting, options_errvslam );
end

%% slam with sp-odo-temp
if bCalibSpOdoTmp
    %% calib with m-slam
    % do m-slam-calib
    solver.SolveJointOptMSlam(measure, calib, map, options_mslam_spodotemp);
    % display v-slam-calib results
    calib.DispCalib;
    % record
    struct_results.mslamodotemp = calib.GetCalibResult;
    % do v-slam and compute error
    solver.SolveJointOptVSlam(measure, calib, map, options_vslam);
    struct_errors.mslamodotemp = Err_vSlam( measure, calib, map, setting, options_errvslam );
    
    %% calib with v-slam
    % do v-slam-calib
    solver.SolveJointOptVSlam(measure, calib, map, options_vslam_spodotemp);
    % display v-slam-calib results
    calib.DispCalib;
    % record
    struct_results.vslamodotemp = calib.GetCalibResult;
    % do v-slam and compute error
    solver.SolveJointOptVSlam(measure, calib, map, options_vslam);
    struct_errors.vslamodotemp = Err_vSlam( measure, calib, map, setting, options_errvslam );
end

%% draw results
options_drawmap = struct('strTitle', 'v-SLAM Result: v-Calib.', 'fileNameFigOut', '.\temp\vslam-final', ...
    'bDrawMeasure', true, 'bDrawMkRot', true, 'scaleMk', 3);
map.DrawMap(measure, calib, setting, options_drawmap);

cell_groups = {'initvo', 'initmk'};
options_drawerr = struct('flag', 1, 'b_saveimg', true, ...
    'outputfilename', '.\temp\err-img-init');
ScatterErr( struct_errors, cell_groups, options_drawerr);
options_drawerr = struct('flag', 2, 'b_saveimg', true, ...
    'outputfilename', '.\temp\err-odoxy-init');
ScatterErr( struct_errors, cell_groups, options_drawerr);
options_drawerr = struct('flag', 3, 'b_saveimg', true, ...
    'outputfilename', '.\temp\err-odoxt-init');
ScatterErr( struct_errors, cell_groups, options_drawerr);

cell_groups = {'initmk', 'mslam', 'mslamodo', 'mslamodotemp'};
options_drawerr = struct('flag', 1, 'b_saveimg', true, ...
    'outputfilename', '.\temp\err-img-mslam');
ScatterErr( struct_errors, cell_groups, options_drawerr);
options_drawerr = struct('flag', 2, 'b_saveimg', true, ...
    'outputfilename', '.\temp\err-odoxy-mslam');
ScatterErr( struct_errors, cell_groups, options_drawerr);
options_drawerr = struct('flag', 3, 'b_saveimg', true, ...
    'outputfilename', '.\temp\err-odoxt-mslam');
ScatterErr( struct_errors, cell_groups, options_drawerr);

cell_groups = {'initmk', 'vslam', 'vslamodo', 'vslamodotemp'};
options_drawerr = struct('flag', 1, 'b_saveimg', true, ...
    'outputfilename', '.\temp\err-img-vslam');
ScatterErr( struct_errors, cell_groups, options_drawerr);
options_drawerr = struct('flag', 2, 'b_saveimg', true, ...
    'outputfilename', '.\temp\err-odoxy-vslam');
ScatterErr( struct_errors, cell_groups, options_drawerr);
options_drawerr = struct('flag', 3, 'b_saveimg', true, ...
    'outputfilename', '.\temp\err-odoxt-vslam');
ScatterErr( struct_errors, cell_groups, options_drawerr);

%% print rmse
cell_groups = { ...
    'initvo', 'initmk', ...
    'mslam', 'mslamodo', 'mslamodotemp', ...
    'vslam', 'vslamodo', 'vslamodotemp'};
PrintRmsErr(struct_errors, cell_groups);
PrintCalibRes(struct_results, cell_groups);















