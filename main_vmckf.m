%% init: create class objs and load data
clear;

[FileName,PathName] = uigetfile('*.xml', 'Select the setting file', 'C:\Workspace\Data\');
settingFile = [PathName, FileName];
% settingFile = 'C:\Workspace\Data\cuhksz-2016.3.15\r2-rightback-bidir-mk127-2016031520\config\setting-simcalibmap3d-vmckf.xml';

configXml = readXml_vmckf(settingFile);

% measure
measure = ClassMeasure(configXml.PathFold, configXml.NameMk, configXml.NameOdo);
measure.ReadRecData;
measure.PruneData(configXml.ThreshTransPruneData, configXml.ThreshRotPruneData);

% init solver
solverRt = ClassSolverRt;

q_c_b = configXml.qcbInit;
sigma_q_c_b = configXml.qcbSigmaInit;
pt3_c_b = configXml.pt3cbInit;
sigma_pt3_c_b = configXml.pt3cbSigmaInit;
solverRt.InitXFull(q_c_b, sigma_q_c_b, pt3_c_b, sigma_pt3_c_b);

% init ground truth
mu_x_true = configXml.MuXTrue;

%% solve in rt, main loop
vecLp = measure.odo.lp;
rec_mu = [];
rec_sigma = cell(0,1);

odoLast = measure.GetOdoLp(vecLp(1));
odoLast.sigma = zeros(3,3);

for i = 1:(numel(vecLp))
    lpNow = vecLp(i);
    
    % get measure now
    odoNow = measure.GetOdoLp(lpNow);
    odoNow = solverRt.FunPrpgOdo(odoLast, odoNow);
    mkNow = measure.GetMkLp(lpNow);
    
    % set measure now
    solverRt.SetMeasNew(odoNow, mkNow);
    
    % refresh calib res
    solverRt.CalibFull;
    
    % renew measure rec
    solverRt.RenewMeasRec;
    
    % record
    rec_mu = [rec_mu; solverRt.x_full.'-mu_x_true.'];
    rec_sigma{end+1, 1} = solverRt.sigma_x_full;
    
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

disp('x_full:');
disp(solverRt.x_full);
disp('error');
disp(solverRt.x_full - mu_x_true);
disp('sigma_x_full:');
disp(solverRt.sigma_x_full);



