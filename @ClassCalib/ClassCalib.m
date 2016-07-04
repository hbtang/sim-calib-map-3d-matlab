classdef ClassCalib < handle
    % class of calibration results, and transform funtions
    % definition of frame cg: camera frame projected on the ground, z axis of
    % camera frame projected onto ground plane and become x axis
    
    properties
        % 3d transform from base frame to camera frame
        T3d_b_c; rvec_b_c; tvec_b_c;
        % 3d transform from base frame to camera ground projected frame
        T3d_b_cg; rvec_b_cg; tvec_b_cg;
        % 3d transform from camera ground projected frame to camera frame
        T3d_cg_c; rvec_cg_c; tvec_cg_c;
        % norm vector of ground plane in camera frame, pointed updard
        pvec_g_c;
        % distance from ground to camera, positive if camera above ground
        dist_g_c;
    end
    methods
        % constructor function
        function this = ClassCalib()
            % init as camera point right (z_c = -y_b)
            this.T3d_b_c = [-1 0 0 0; 0 0 -1 0; 0 -1 0 0; 0 0 0 1];
            RefreshByTbc(this);
            
            % init as camera point right with u point upward
%             this.T3d_b_c = [0 1 0 0; 0 0 -1 0; -1 0 0 0; 0 0 0 1];
%             RefreshByTbc(this);
            
        end
        
        % refresh calibration properties according to _b_c
        RefreshByTbc(this); RefreshByVecbc(this);
        
        % refresh calibration properties according to _b_cg and _cg_c
        RefreshByTbcgc(this); RefreshByVecbcgc(this);
        
        % refresh calibration properties according to pvec_g_c and dist_g_c
        RefreshByGrnd(this);
        
        % get ps2d_b_cg
        function ps2d_b_cg = GetPs2dbcg(this)
            ps2d_b_cg = zeros(3,1);
            ps2d_b_cg(3) = this.rvec_b_cg(3);
            ps2d_b_cg(1:2) = this.tvec_b_cg(1:2);
        end
        
        % set ps2d_b_cg
        function SetPs2dbcg(this, ps2d_b_cg)
            this.rvec_b_cg = [0;0;ps2d_b_cg(3)];
            this.tvec_b_cg = [ps2d_b_cg(1:2);0];
            RefreshByVecbcgc(this);
        end
        
        % display ps2d_b_cg
        function DispPs2dbcg(this)
            ps2d_b_cg = this.GetPs2dbcg;
            disp(['Current estimated ps2d_b_cg: ', num2str(ps2d_b_cg(1)), ' ', ...
                num2str(ps2d_b_cg(2)), ' ', num2str(ps2d_b_cg(3)), ' ']);
            disp(' ');
        end
    end
end

