classdef ClassSimulator < handle
    %CLASSSIMULATOR class of simulator
    % to generate record file according to a predefined map
    
    properties
        % io config
        % input file of map, including landmark info
        InputFileMapId;
        % input file of camera configure, including camera parameters: T_b_c, FoV...
        InputFileCamId;
        % output file to record
        OutputFileOdoId; OutputFileMkId;
        % figure handles
        hds;
        % status if quit main process
        ifQuit;
        
        % simulated environment
        % map: ClassMap
        map;
        % calib: ClassCalib
        calib;
        % camera configure
        config;
        
        % robot status
        % current loop id
        lp;
        % current robot pose in w frame, single variable
        ps2d_w_b;
        % odometry measurements, with error ratio (proportional to measure)
        odo;
        %         stdErrRatioOdoLin; stdErrRatioOdoRot;
        % current mark measure, vector possible
        mk;
        % current robot velocity
        velLin; velRot;
        
    end
    
    methods
        function this = ClassSimulator()
            this.lp = 0;
            this.ps2d_w_b = [0;0;0];
            this.velLin = 0; this.velRot = 0;
            
            this.odo = struct('lp',0,'x',0,'y',0,'theta',0);
            this.mk = struct('lp',[],'id',[],'rvec',[],'tvec',[]);
            
            this.config = struct('angleOfViewH',[],'angleOfViewV',[],...
                'distRangeMin',[],'distRangeMax',[]);
            this.map = ClassMap;
            this.calib = ClassCalib;
            
            this.hds = struct('hdFigSim', [], 'hdObjBaseX', [], ...
                'hdObjBaseY', [], 'hdObjBaseZ', [], 'hdObjMap', [], 'hdObjRange', []);
            
            this.ifQuit = false;
            
        end
        % init function, load map and calib info, set output path, ...
        init(this);
        % main function of simulator
        run(this);
        % stop function
        stop(this);
        % draw or refresh current status
        draw(this);
        % write into record file
        record(this);
        % call back function when key pressed
        onKeyPressed(this, hdFig, callBackData);
        % renew mark observation
        observe(this);
        % add noise into rec file
        AddNoise2Rec(this, noiseConfig, foldPath, setId);
    end
    
end

