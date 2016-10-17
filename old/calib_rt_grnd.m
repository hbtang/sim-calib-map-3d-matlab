%% init: create class objs and load data
clear;

% measure
PathFold = './data/rec_p1_right_circle_mk120_2016031216';
NameMk = 'Mk.rec'; 
NameOdo = 'Odo.rec';

measure = ClassMeasure(PathFold, NameMk, NameOdo);
measure.ReadRecData;
measure.PruneData(200, 4*pi/180);

% solver
solverRt = ClassSolverRt;

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
    solverRt.CalibGrnd;
        
    % renew measure rec
    solverRt.RenewMeasRec;
    
    % record
    rec_mu = [rec_mu; solverRt.pl_g_c.'];
    rec_sigma{end+1,1} = solverRt.sigma_pl_g_c;    
    
    % renew odoLast
    odoLast = odoNow;
    
    % display info
    disp(['loop: ', num2str(i)]);
end

%% draw results
DrawPlotEnv( rec_mu, rec_sigma );



