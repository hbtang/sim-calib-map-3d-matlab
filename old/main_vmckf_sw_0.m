% main_vmckf_sw: run vmckf with sliding-window

%% init: create class objs and load data
clear;

% read configure file
% [FileName,PathName] = uigetfile('*.xml', 'Select the setting file', 'C:\Workspace\Data\');
% settingFile = [PathName, FileName];
% settingFile = 'C:\Workspace\Data\cuhksz-2016.3.15\r2-rightback-bidir-mk127-2016031520\config\setting-simcalibmap3d-vmckf.xml';
settingFile = 'C:\Workspace\Data\sim\sim-sqrmap-inout-2016.11.8\config\setting-simcalibmap3d-vmckf.xml';
configXml = readXml_vmckf(settingFile);

% read record data
measure = ClassMeasure(configXml.PathFold, configXml.NameMk, configXml.NameOdo);
measure.ReadRecData;
%%%%%%%% Debug Begin %%%%%%%%
% reset t_odo, because in AGV dataset bad match in t_odo
measure.time.t_odo = measure.time.t_mk;
% measure.time.t_odo = TimeOdo_ConstVel(measure.time.t_odo);
% measure.time.t_odo = TimeOdo_EqualOffset(measure.time.t_odo, measure.odo);
%%%%%%%% Debug End  %%%%%%%%%
measure_raw = ClassMeasure();
measure.CopyTo(measure_raw);
measure.PruneData(configXml.ThreshTransPruneData, configXml.ThreshRotPruneData);
% compute velocity by interpolation
measure.odo = Odo_Interpolate(measure.odo, measure_raw.odo, measure_raw.time, 1);
measure_raw.odo = Odo_Interpolate(measure_raw.odo, measure_raw.odo, measure_raw.time);

% init calib
calib = ClassCalib;
pt3cbInit = configXml.pt3cbInit;
qcbInit = configXml.qcbInit;
RcbInit = quat2rot(qcbInit);
rveccbInit = rodrigues(RcbInit);
calib.SetVecbc(rveccbInit, pt3cbInit);

% read error config, todo...
errConfig.stdErrRatioOdoLin = configXml.SolverConfig_StdErrRatioOdoLin;
errConfig.stdErrRatioOdoRot = configXml.SolverConfig_StdErrRatioOdoRot;
errConfig.MinStdErrOdoLin = configXml.SolverConfig_MinStdErrOdoLin;
errConfig.MinStdErrOdoRot = configXml.SolverConfig_MinStdErrOdoRot;
errConfig.stdErrRatioMkX = configXml.SolverConfig_StdErrRatioMkX;
errConfig.stdErrRatioMkY = configXml.SolverConfig_StdErrRatioMkY;
errConfig.stdErrRatioMkZ = configXml.SolverConfig_StdErrRatioMkZ;

% init solver
solver = ClassSolverVmckfSw(errConfig);
solver.SetStateFromCalib(calib, 1, true);
% init ground truth
mu_x_true = configXml.MuXTrue;

%% solve in rt, main loop
vec_lp = measure.odo.lp;
rec_mu_x = [];
for i = 1:(numel(vec_lp))
    
    lp_now = vec_lp(i);
    cell_batch = solver.CreateBatch(lp_now, measure, solver.struct_slidingWindow);
    
    if numel(cell_batch) ~= 0
        % for each batch: multi instants, same mark id
        for j = 1:numel(cell_batch)            
            cell_cnstr = solver.CreateCnstr( cell_batch{j}, measure );
            
            % for each constraint: two instants, one mark id
            for k = 1:numel(cell_cnstr)
                
                % read
                struct_cnstr = cell_cnstr{k};
                vec_mu_x = solver.vec_mu_x;
                mat_Sigma_x = solver.mat_Sigma_x;
                
                % measurement
                [vec_mu_y, mat_Sigma_y] = solver.Measure(struct_cnstr);
                
                % compute Hessian, Jacobian and Cost
                [vec_Cost, mat_Jacobian, cellmat_Hessian] = solver.CostApproxCnstr(struct_cnstr, vec_mu_x, vec_mu_y);
                
                % virtual measurement
                [vec_mu_z, mat_Sigma_z] = solver.VirtualMeasure(mat_Sigma_x, mat_Sigma_y, cellmat_Hessian);
                
                % correct
                [vec_mu_x_new, mat_Sigma_x_new] = solver.Correct(vec_mu_x, mat_Sigma_x, vec_mu_y, mat_Sigma_y, vec_mu_z, mat_Sigma_z, vec_Cost, mat_Jacobian);
                
                % set
                solver.vec_mu_x = vec_mu_x_new;
                solver.mat_Sigma_x = mat_Sigma_x_new;
            end            
        end
    end
    rec_mu_x = [rec_mu_x; solver.vec_mu_x.'];
    
    solver.struct_slidingWindow = solver.RenewSlidingWindow(lp_now, measure, solver.struct_slidingWindow);
    
end




