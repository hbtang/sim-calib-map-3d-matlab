simulator = ClassSimulator;

%% single set
% noiseConfig.stdErrRatioOdoLin = 0.01;
% noiseConfig.stdErrRatioOdoRot = 0.01;
% noiseConfig.stdErrRatioMkDpth = 0.01;
% noiseConfig.stdErrRatioMkLtr = 0.01;
% simulator.AddNoise2Rec(noiseConfig, '../data/sim-sqrmap-inout-20160118/');

%% multiple sets
noiseConfig.stdErrRatioOdoLin = 0.01;
noiseConfig.stdErrRatioOdoRot = 0.01;
noiseConfig.stdErrRatioMkDpth = 0.01;
noiseConfig.stdErrRatioMkLtr = 0.01;

for i = 6:10
    noiseConfig.stdErrRatioOdoLin = 0.01*i;
    noiseConfig.stdErrRatioOdoRot = 0.01*i;
    noiseConfig.stdErrRatioMkDpth = 0.01*i;
    noiseConfig.stdErrRatioMkLtr = 0.01*i;
    
    for k = 1:20
        simulator.AddNoise2Rec(noiseConfig, '../data/sim-sqrmap-inout-20160118/', k);        
        disp(['i:', num2str(i),' k:' num2str(k)]);
    end
end
