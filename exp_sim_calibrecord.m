% do calib and record results, by simulation dataset: initguo, init, mslam, vslam
clear;
close all;

%% init
set_odo = 1;
set_img = 1;
set_st = 1;
set_end = 20;

stdratio_odo_st = 0;
stdratio_odo_end = 0.04;
stdratio_odo_step = 0.001;

std_img_st = 0;
std_img_end = 1;
std_img_step = 1;

bDoRecord = true;
bCalibSp = true;
bCalibSpOdo = true;
bCalibSpOdoTmp = true;


%% do calib. loop
for set = set_st:set_end
    
    set_odo = set;
    set_img = set;
    for stdratio_odo = stdratio_odo_st:stdratio_odo_step:stdratio_odo_end
        
        stdratio_lin = stdratio_odo;
        stdratio_rot = stdratio_odo;
        str_suffix_odo = [ ...
            '-el', num2str(stdratio_lin), ...
            '-er', num2str(stdratio_rot), ...
            '-s', num2str(set_odo), ...
            ];
        odofilename = ['Odo', str_suffix_odo, '.rec'];
        
        for std_img = std_img_st:std_img_step:std_img_end
            
            std_imgu = std_img;
            std_imgv = std_img;
            str_suffix_mk = [ ...
                '-eu', num2str(std_imgu), ...
                '-ev', num2str(std_imgv), ...
                '-s', num2str(set_img), ...
                ];
            markfilename = ['Mk', str_suffix_mk, '.rec'];
            
            str_file_record = ['.\temp\results', str_suffix_odo, str_suffix_mk, '.mat'];
            if exist(str_file_record, 'file')
                continue;
            end
            
            %% init: create class objs and load data
            % setting
            setting = YAML.read('setting-slam-sim.yml');
            setting.path.odofilename = odofilename;
            setting.path.markfilename = markfilename;
            setting.error.odo.stdratio_lin = stdratio_lin;
            setting.error.odo.stdratio_rot = stdratio_rot;
            setting.error.mk.std_imgu = std_imgu;
            setting.error.mk.std_imgv = std_imgv;
            
            % measure
            measure = ClassMeasure(setting.path.fold, setting.path.markfilename, setting.path.odofilename);
            measure.ReadRecData;
            measure_raw = ClassMeasure();
            measure.CopyTo(measure_raw);
            measure.PruneData(setting.prune.thresh_lin, setting.prune.thresh_rot);
            % compute velocity by interpolation
            measure.odo = Odo_Interpolate(measure.odo, measure_raw.odo, measure_raw.time, 1);
            
            % solver
            solver = ClassSolverSlam(setting);
            
            % map
            map = ClassMap;
            
            % calib
            calib = ClassCalib(setting);
            
            % slam configure
            options_mslam = struct(...
                'bCalibExtRot', true, 'bCalibExtLin', true,...
                'bCalibTmp', false, 'bCalibOdo', false);
            options_vslam = struct(...
                'bCalibExtRot', true, 'bCalibExtLin', true,...
                'bCalibTmp', false, 'bCalibOdo', false, ...
                'bCalibCamMat', false, 'bCalibCamDist', false);
            
            % record
            struct_record = [];
            struct_record.stdratio_lin = stdratio_lin;
            struct_record.stdratio_rot = stdratio_rot;
            struct_record.std_imgu = std_imgu;
            struct_record.std_imgv = std_imgv;
            
            
            %% init by guo's solution
            solver.SolveInitGuo(measure, calib);
            calib.DispCalib;
            struct_record.result_initvo = calib.GetCalibResult;
            
            %% init by my solution
            solver.SolveGrndPlaneLin(measure, calib);
            solver.SolveYawXY(measure, calib);
            calib.DispCalib;
            struct_record.result_initmk = calib.GetCalibResult;
            
            if bCalibSp
                %% calib with m-slam
                % init. map from odometry and mark observation
                map.InitMap(measure, calib);
                % do m-slam-calib
                solver.SolveJointOptMSlam(measure, calib, map, options_mslam);
                % display v-slam-calib results
                calib.DispCalib;
                % record
                struct_record.result_mslam = calib.GetCalibResult;
                
                %% calib with v-slam
                % do v-slam-calib
                solver.SolveJointOptVSlam(measure, calib, map, options_vslam);
                % display v-slam-calib results
                calib.DispCalib;
                % record
                struct_record.result_vslam = calib.GetCalibResult;
            end
            
            if bCalibSpOdo
                %% calib with m-slam
                options_mslam.bCalibOdo = true;
                % do m-slam-calib
                solver.SolveJointOptMSlam(measure, calib, map, options_mslam);
                % display v-slam-calib results
                calib.DispCalib;
                % record
                struct_record.result_mslamodo = calib.GetCalibResult;
                
                %% calib with v-slam
                options_vslam.bCalibOdo = true;
                % do v-slam-calib
                solver.SolveJointOptVSlam(measure, calib, map, options_vslam);
                % display v-slam-calib results
                calib.DispCalib;
                % record
                struct_record.result_vslamodo = calib.GetCalibResult;
            end
            
            if bCalibSpOdoTmp
                %% calib with m-slam
                options_mslam.bCalibOdo = true;
                options_mslam.bCalibTmp = true;
                % do m-slam-calib
                solver.SolveJointOptMSlam(measure, calib, map, options_mslam);
                % display v-slam-calib results
                calib.DispCalib;
                % record
                struct_record.result_mslamodotemp = calib.GetCalibResult;
                
                %% calib with v-slam
                options_vslam.bCalibOdo = true;
                options_vslam.bCalibTmp = true;
                % do v-slam-calib
                solver.SolveJointOptVSlam(measure, calib, map, options_vslam);
                % display v-slam-calib results
                calib.DispCalib;
                % record
                struct_record.result_vslamodotemp = calib.GetCalibResult;
            end
            
            %% record
            if bDoRecord
                save(str_file_record, 'struct_record');
            end
        end
    end
    
end
