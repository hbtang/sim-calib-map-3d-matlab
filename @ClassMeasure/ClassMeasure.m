classdef ClassMeasure < handle
    properties
        % io configures
        InputFolderPath;
        InputMkFilePath;
        InputOdoFilePath;
        
        % measurement data
        odo;
        mk;
        time;
        
    end
    
    methods
        % constructor function, set input file path
        function this = ClassMeasure(PathFold, NameMk, NameOdo)
            this.mk = struct('lp', [], 'id', [], ...
                'rvec', [], 'tvec', [], 'num', [], ...
                'numMkId', [], 'vecMkId', []);
            this.odo = struct('lp',[], 'x',[],'y',[],'theta',[], ...
                'num', []);
            this.time = struct('lp',[],'t_odo',[],'t_mk',[]);
            if nargin == 1
                this.InputFolderPath = PathFold;
                this.InputMkFilePath = [PathFold, '/rec/Mk.rec'];
                this.InputOdoFilePath = [PathFold, '/rec/Odo.rec'];                
            elseif nargin == 3              
                this.InputFolderPath = PathFold;
                this.InputMkFilePath = [PathFold, '/rec/', NameMk];
                this.InputOdoFilePath = [PathFold, '/rec/', NameOdo];                   
            end
        end
        
        function CopyTo(this, copy)
            copy.mk = this.mk;
            copy.odo = this.odo;
            copy.time = this.time;
            copy.InputFolderPath = this.InputFolderPath;
            copy.InputMkFilePath = this.InputMkFilePath;
            copy.InputOdoFilePath = this.InputOdoFilePath;            
        end
        
        % function read input file
        ReadRecData(this);
        PruneData( this, threshDistOdo, threshAngleOdo );
        PruneDataByVecLp(this, vecLp);
        
        % function get measurement of lp
        odoLp = GetOdoLp(this, lp);
        mkLp = GetMkLp(this, lp);
    end
end

