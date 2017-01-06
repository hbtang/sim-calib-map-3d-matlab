%% record with noise
load('./sim/simulator.mat');
simulator.Init;

%% debug: add temporal-odometric-cameraintrinsic errors
simulator.calib.dt = 0.01;
simulator.calib.k_odo_lin = 1.01;
simulator.calib.k_odo_rot = 1.01;
% simulator.calib.mat_camera = [480 0 300; 0 480 220; 0 0 1];
% simulator.calib.vec_distortion = [0 0 0 0 0];
% set = 2;

%% set=1, no error; set=2, temp-odo-cam error
for set = 1:20
    
    %% record mk
%     for stdimg = 0:0.025:1
%         
%         simulator.setting.error.mk.std_imgu = stdimg;
%         simulator.setting.error.mk.std_imgv = stdimg;
%         
%         std_imgu = simulator.setting.error.mk.std_imgu;
%         std_imgv = simulator.setting.error.mk.std_imgv;
%         simulator.mk_noise = NoiseMk(simulator, simulator.mk_true, simulator.calib, simulator.setting);
%         str_suffix_mk = [ ...
%             '-eu', num2str(std_imgu), ...
%             '-ev', num2str(std_imgv), ...
%             '-s', num2str(set), ...
%             ];
%         
%         options = struct( ...
%             'str_suffix_mk', str_suffix_mk, ...
%             'b_record_mk', true );
%         simulator.Record(options);
%         
%         disp(['set: ', num2str(set), ', stdimg: ' , num2str(stdimg)]);
%     end
    
    %% record odo
    for stdratio = 0:0.001:0.04
        simulator.setting.error.odo.stdratio_lin = stdratio;
        simulator.setting.error.odo.stdratio_rot = stdratio;
        
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
        
        disp(['set: ', num2str(set), ', stdrodo: ' , num2str(stdratio)]);
    end
    
end