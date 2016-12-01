%% record with noise
load('./sim/simulator.mat');
simulator.Init;

%% record mk
for stdimg = 0:0.1:3
    simulator.setting.error.mk.std_imgu = stdimg;
    simulator.setting.error.mk.std_imgv = stdimg;
    numSet = 1;
    for set = 1:numSet
        std_imgu = simulator.setting.error.mk.std_imgu;
        std_imgv = simulator.setting.error.mk.std_imgv;
        simulator.mk_noise = NoiseMk(simulator, simulator.mk_true, simulator.calib, simulator.setting);
        str_suffix_mk = [ ...
            '-eu', num2str(std_imgu), ...
            '-ev', num2str(std_imgv), ...
            '-s', num2str(set), ...
            ];
        
        options = struct( ...
            'str_suffix_mk', str_suffix_mk, ...
            'b_record_mk', true );
        simulator.Record(options);
    end
    stdimg
end

%% record odo
for stdratio = 0:0.001:0.03
    simulator.setting.error.odo.stdratio_lin = stdratio;
    simulator.setting.error.odo.stdratio_rot = stdratio;
    numSet = 1;
    for set = 1:numSet
        
        simulator.odo_noise = NoiseOdo(simulator, simulator.odo_true, simulator.calib, simulator.setting);
        stdratio_lin = simulator.setting.error.odo.stdratio_lin;
        stdratio_rot = simulator.setting.error.odo.stdratio_rot;
        
        str_suffix_odo = [ ...
            '-el', num2str(stdratio_lin), ...
            '-er', num2str(stdratio_rot), ...
            '-s', num2str(set), ...
            ];
        
        options = struct( ...
            'str_suffix_odo', str_suffix_odo, ...
            'b_record_odo', true );
        simulator.Record(options);
    end
    stdratio
end