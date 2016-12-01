classdef ClassSimulator < handle
    %CLASSSIMULATOR class of simulator
    % to generate record file according to a predefined map
    
    properties
        %% configuration
        % io configures
        files = struct( ...
            'str_simfolder', [], ...
            'file_out_odo', [], ...
            'file_out_mk', [], ...
            'str_path_setting', []);
        % handles of figure and figure objects
        hds = struct( ...
            'hdFigSim', [], ...
            'hdObjBaseX', [], 'hdObjBaseY', [], 'hdObjBaseZ', [], ...
            'hdObjMap', [], 'hdObjRange', []);
        % status
        flag = struct( ...
            'bQuit', false);
        % setting configuration read from yaml file
        setting;
        
        %% simulation environment
        % map: ClassMap
        map;
        % calib: ClassCalib
        calib;
        % frequency configure
        timestep;
        
        %% robot status
        % current loop id
        lp;
        % current robot pose in w frame, single variable
        ps2d_w_b;
        % odometry measurements, with error ratio (proportional to measure)
        odo;
        % current mark measure, vector possible
        mk;
        % current robot velocity
        vel_lin; vel_rot;
        
        %% raw data record
        odo_true;
        mk_true;
        odo_noise;
        mk_noise;
        
    end
    
    methods
        function this = ClassSimulator()
            %% robot status
            this.lp = 0;
            this.ps2d_w_b = [0;0;0];
            this.vel_lin = 0;
            this.vel_rot = 0;
            this.odo = struct('lp',0,'x',0,'y',0,'theta',0);
            this.mk = struct('lp',[],'id',[],'rvec',[],'tvec',[]);
            
            %% raw data record
            this.odo_true = struct('lp',0,'x',0,'y',0,'theta',0);
            this.mk_true = struct('lp',[],'id',[],'rvec',[],'tvec',[],'image',[]);
            this.odo_noise = struct('lp',0,'x',0,'y',0,'theta',0);
            this.mk_noise = struct('lp',[],'id',[],'rvec',[],'tvec',[],'image',[]);
            
            %% simulation environment
            this.map = ClassMap;
            this.calib = ClassCalib;
            this.timestep = 0.1;
            
            %% configuration
            this.files = struct( ...
                'str_simfolder', [], ...
                'file_out_odo', [], ...
                'file_out_mk', [], ...
                'str_path_setting', []);
            this.hds = struct( ...
                'hdFigSim', [], ...
                'hdObjBaseX', [], 'hdObjBaseY', [], 'hdObjBaseZ', [], ...
                'hdObjMap', [], 'hdObjRange', []);
            this.flag = struct( ...
                'bQuit', false);
            this.setting = [];
        end
        
        % init function, load map and calib info, set output path, ...
        Init(this);
        % main function of simulator
        Run(this);
        % stop function
        Stop(this);
        % draw or refresh current status
        Draw(this);
        % renew mark observation
        Observe(this);
        
        % create odo_out from odo_true with noise
        odo_out = NoiseOdo(this, odo_true, calib, setting);
        mk_out = NoiseMk(this, mk_true, calib, setting);
        
        % write odo and mk into record file
        Record( this, options )
        
        % call back function when key pressed
        OnKeyPressed(this, hdFig, callBackData);
        
        % add noise into rec file
        AddNoise2Rec(this, noiseConfig, foldPath, setId);
    end
    
end

