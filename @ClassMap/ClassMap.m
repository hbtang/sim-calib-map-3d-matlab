classdef ClassMap < handle
    % class of map
    properties
        % keyframes
        kfs;
        % marks
        mks;
        % points
        pts;
        % figure handle for map display
        hdFigMap;
        
    end
    methods
        % constructor function
        function this = ClassMap()
            this.kfs = struct('lp',[],'rvec_w_b',[],'tvec_w_b',[],...
                'ps2d_w_b',[]);
            this.mks = struct('id',[],'rvec_w_m',[],'tvec_w_m',[]);
            this.pts = struct('idMk',[],'idPt',[],'tvec_w_p',[]);            
        end
        % draw current map
        DrawMap(this); DrawMapWithMeasure(this, measure, calib);
        % init map from measurement
        InitMap(this, measure, calib);
        % refresh keyframe by ps2d_w_b
        function RefreshKfsByPs2dwb(this)
            this.kfs.rvec_w_b(:,3) = this.kfs.ps2d_w_b(:,3);
            this.kfs.tvec_w_b(:,1:2) = this.kfs.ps2d_w_b(:,1:2);
        end        
        
    end
end



