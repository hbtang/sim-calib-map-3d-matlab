%% run and save true data

clear;
simulator = ClassSimulator;
simulator.Init;
simulator.Run;
save('./sim/simulator.mat');
simulator.Stop;
