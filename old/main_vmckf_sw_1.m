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
measure.odo = Odo_Interpolate(measure.odo, measure_raw.odo, measure_raw.time);
measure_raw.odo = Odo_Interpolate(measure_raw.odo, measure_raw.odo, measure_raw.time);

% init solver
solverRt = ClassSolverRt;
q_c_b = configXml.qcbInit;
sigma_q_c_b = configXml.qcbSigmaInit;
pt3_c_b = configXml.pt3cbInit;
sigma_pt3_c_b = configXml.pt3cbSigmaInit;

q_c_b = [0;0;1;0];
solverRt.InitXFull(q_c_b, sigma_q_c_b, pt3_c_b, sigma_pt3_c_b);

% init ground truth
mu_x_true = configXml.MuXTrue;

%% debug: with sliding window
% vec_lp = measure.odo.lp;
% count = 0;
% for i = 1:numel(vec_lp);
%     lp_now = vec_lp(i);
%     cell_batch = solverRt.CreateBatch(lp_now, measure);
%     
%     %%%% todo: add filter here ... %%%%    
%     
%     solverRt.RefreshSlidingWindow( lp_now, measure );
% end


%% solve in rt, main loop
vec_lp = measure.odo.lp;
rec_mu = [];
rec_sigma = cell(0,1);
rec_dt = [];

odoLast = measure.GetOdoLp(vec_lp(1));
odoLast.sigma = zeros(3,3);

for i = 1:(numel(vec_lp))
    lpNow = vec_lp(i);
    
    % get measure now
    odoNow = measure.GetOdoLp(lpNow, true);
    odoNow = solverRt.FunPrpgOdo(odoLast, odoNow);
    mkNow = measure.GetMkLp(lpNow);
    
    % set measure now
    solverRt.SetMeasNew(odoNow, mkNow);
    
    % refresh calib res
    solverRt.CalibFullDelay;
    
    % renew measure rec
    solverRt.RenewMeasRec(true);
    
    % record
    rec_mu = [rec_mu; solverRt.x_full.'-mu_x_true.'];
    rec_sigma{end+1, 1} = solverRt.sigma_x_full;
    rec_dt = [rec_dt; solverRt.dt sqrt(solverRt.sigma_x_full_dt(end,end))];
    
    % renew odoLast
    odoLast = odoNow;
    
    % display info
    %     disp(['loop: ', num2str(i)]);
end

%% draw results
vec_id = [1;2;3;4];
DrawPlotEnv( rec_mu, rec_sigma, vec_id );

vec_id = [5;6];
DrawPlotEnv( rec_mu, rec_sigma, vec_id );

figure;
plot(rec_dt);

disp('x_full:');
disp(solverRt.x_full);
disp('error');
disp(solverRt.x_full - mu_x_true);
disp('sigma_x_full:');
disp(solverRt.sigma_x_full);



