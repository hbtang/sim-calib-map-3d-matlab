%% init: create class objs and load data
clear;

% measure
PathFold = './data/rec_r2_rightback_bidir_mk127_2016031520';
NameMk = 'Mk.rec';
NameOdo = 'Odo.rec';
% NameMk = 'MkNs-0.05-0.01.rec';
% NameOdo = 'OdoNs-0.05-0.01.rec';

measure = ClassMeasure(PathFold, NameMk, NameOdo);
measure.ReadRecData;
measure.PruneData(200, 4*pi/180);

% init solver
solverRt = ClassSolverRt;
% q_c_b = [1;0;0;0];
q_c_b = [0;0;1/sqrt(2);-1/sqrt(2)];
sigma_q_c_b = diag([3;3;3;3]);
pt3_c_b = [100;100;0];
sigma_pt3_c_b = diag([1e6;1e6;1e6]);
solverRt.InitXFull(q_c_b, sigma_q_c_b, pt3_c_b, sigma_pt3_c_b);

% init ground truth, for simulator dataset
% for rec_r2_rightback_bidir_mk127_2016031520
mu_x_true = [0.1499; 0.1490; 0.6803; -0.7018; 175.5707; -289.4186];
% mu_x_true = [0;0;0;0;0;0];
% mu_x_true = [0;0;1/sqrt(2);-1/sqrt(2);0;0];
% mu_x_true = [0.0062;0.0010;0.6304;-0.7762;144.6295;-436.894];


%% debuging: refine map
% measure.odo = FunRefineOdo(measure.odo, 1.0, 1.0);

% for rec_r2_right_anticlockx3_20160119
% measure.odo = FunRefineOdo(measure.odo, 1.0015, 0.9975);

% for rec_p1_right_clock_mk120_2016031509
% measure.odo = FunRefineOdo(measure.odo, 1, 0.945);

% for rec_p1_right_anticlock_mk120_2016031422
% measure.odo = FunRefineOdo(measure.odo, 1, 0.97);

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



