clear;

PathFold = '../data/sim-sqrmap-inout-20160118';

MdNs = 0; OdoNs = 0; SetId = 1;
stdRatioMkZ = 0.01*MdNs;
stdRatioMkXY = 0.01*MdNs;
stdRatioOdoL = 0.01*OdoNs;
stdRatioOdoR = 0.01*OdoNs;

NameMk = ['Mk-z', num2str(stdRatioMkZ*100), '-xy', num2str(stdRatioMkXY*100), ...
    '-s', num2str(SetId), '.rec'];
NameOdo = ['Odo-l', num2str(stdRatioOdoL*100), '-r', num2str(stdRatioOdoR*100), ...
    '-s', num2str(SetId), '.rec'];

if MdNs == 0
    NameMk = ['Mk-z', num2str(stdRatioMkZ*100), '-xy', num2str(stdRatioMkXY*100), ...
        '-s', num2str(1), '.rec'];
end
if OdoNs == 0
    NameOdo = ['Odo-l', num2str(stdRatioOdoL*100), '-r', num2str(stdRatioOdoR*100), ...
        '-s', num2str(1), '.rec'];
end

measure = ClassMeasure(PathFold, NameMk, NameOdo);
measure.ReadRecData;
measure.PruneData(200, 4*pi/180);

vecLpPrune = measure.odo.lp;

%% main for loop

for MdNs = 0:5
     for OdoNs = 0:5
        for SetId = 1:1
            
            %% read dataset and init measure
            stdRatioMkZ = 0.01*MdNs;
            stdRatioMkXY = 0.01*MdNs;
            stdRatioOdoL = 0.01*OdoNs;
            stdRatioOdoR = 0.01*OdoNs;
            
            NameMk = ['Mk-z', num2str(stdRatioMkZ*100), '-xy', num2str(stdRatioMkXY*100), ...
                '-s', num2str(SetId), '.rec'];
            NameOdo = ['Odo-l', num2str(stdRatioOdoL*100), '-r', num2str(stdRatioOdoR*100), ...
                '-s', num2str(SetId), '.rec'];
            
            if MdNs == 0
                NameMk = ['Mk-z', num2str(stdRatioMkZ*100), '-xy', num2str(stdRatioMkXY*100), ...
                    '-s', num2str(1), '.rec'];
            end
            if OdoNs == 0
                NameOdo = ['Odo-l', num2str(stdRatioOdoL*100), '-r', num2str(stdRatioOdoR*100), ...
                    '-s', num2str(1), '.rec'];
            end
            
            measure = ClassMeasure(PathFold, NameMk, NameOdo);
            measure.ReadRecData;
            measure.PruneDataByVecLp(vecLpPrune);
            
            %% multiple loops to calibrate with different initial guess
            for InitId = 1:50
                
                NameMkStr = ['Mk-z', num2str(stdRatioMkZ*100), '-xy', num2str(stdRatioMkXY*100), ...
                    ];
                NameOdoStr = ['Odo-l', num2str(stdRatioOdoL*100), '-r', num2str(stdRatioOdoR*100), ...
                    '-s', num2str(SetId)];                
                PathFoldRec = [PathFold, '/res/'];
                PathFullRec = [PathFoldRec,'/res-',NameMkStr, '-', NameOdoStr, '-i', num2str(InitId), '.mat'];
                
                if exist(PathFullRec, 'file') == 2
                    continue;
                end                
                
                %% init solver
                solverRt = ClassSolverRt;
                
                solverRt.errConfig.errRatioOdoTr = stdRatioOdoL;
                solverRt.errConfig.errRatioOdoRot = stdRatioOdoR;
                solverRt.errConfig.errRatioMkDpt = stdRatioMkZ;
                solverRt.errConfig.errRatioMkLtr = stdRatioMkXY;
                
%                 solverRt.errConfig.errRatioMkDpt = 0.02;
%                 solverRt.errConfig.errRatioMkLtr = 0.02;
                                
                q_c_b = normrnd([0;0;1/sqrt(2);-1/sqrt(2)], [0.3;0.3;0.3;0.3]);
                pt3_c_b = normrnd([0; 0; 0], [500; 500 ;0]);
                
                sigma_q_c_b = diag([3;3;3;3]);
                sigma_pt3_c_b = diag([1e6;1e6;1e6]);
                solverRt.InitXFull(q_c_b, sigma_q_c_b, pt3_c_b, sigma_pt3_c_b);
                
                %% set ground truth, for simulator dataset
                
                % for simulation dataset sim_ ...
                mu_x_true = [0;0;1/sqrt(2);-1/sqrt(2);0;0];
                
                %% solve in rt, main loop
                vecLp = measure.odo.lp;
                rec_err = []; rec_sigma = cell(0,1);
                rec_x = []; rec_lp = []; rec_dt = [];
                
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
                    tic;
                    solverRt.CalibFull;
                    dt = toc;
                    % renew measure rec
                    solverRt.RenewMeasRec;
                    
                    % determine direction of q_c_b
                    if solverRt.x_full(1:4).'*mu_x_true(1:4) < 0
                        solverRt.x_full(1:4) = -solverRt.x_full(1:4);
                    end
                    
                    % record
                    rec_err = [rec_err; solverRt.x_full.'-mu_x_true.'];
                    rec_sigma{end+1, 1} = solverRt.sigma_x_full;
                    rec_x = [rec_x; solverRt.x_full.'];
                    rec_lp = [rec_lp; lpNow];
                    rec_dt = [rec_dt; dt];
                    
                    % renew odoLast
                    odoLast = odoNow;                    
                end                
                
                %% save results
                save(PathFullRec, ...
                    'rec_lp', 'rec_x', 'rec_sigma','rec_dt');
                
                %% show results
%                 vec_id = [1;2;3;4];     DrawPlotEnv( rec_err, rec_sigma, vec_id );
%                 vec_id = [5;6];         DrawPlotEnv( rec_err, rec_sigma, vec_id );
                %
                %                 disp('x_full:');
                %                 disp(solverRt.x_full);
                %                 disp('sigma_x_full:');
                %                 disp(solverRt.sigma_x_full);
                
                disp(['MdNs:', num2str(MdNs), '; OdoNs:', num2str(OdoNs),...
                    '; SetId:',num2str(SetId), '; CntIter:', num2str(InitId)])
                disp(['error: ', num2str(rec_err(end,:))]);
                
            end
        end
     end
end



