function ReadRecData(this)
%READRECDATA read record data from Mk.rec and Odo.rec

%% reading mark record data
disp(['Reading mark record data from "', this.InputMkFilePath, '"...']);
markFileId = fopen(this.InputMkFilePath,'r');
this.mk = struct('lp', [], 'id', [], 'rvec', [], 'tvec', [], ...
    'num', [], 'numMkId', [], 'vecMkId', []);

% read loop
while ~feof(markFileId)
    lineTmp = fgetl(markFileId);
    if lineTmp(1) == '#'
        continue;
    end
    lineTmp = str2num(lineTmp);
    if lineTmp(2) == 1023
        continue;
    end
        
    this.mk.lp = [this.mk.lp; lineTmp(1)];
    this.mk.id = [this.mk.id; lineTmp(2)];
    this.mk.rvec = [this.mk.rvec; lineTmp(3:5)];
    this.mk.tvec = [this.mk.tvec; lineTmp(6:8)];
end
fclose(markFileId);
this.mk.num = numel(this.mk.lp);
this.mk.vecMkId = unique(this.mk.id);
this.mk.numMkId = numel(this.mk.vecMkId);
disp(['All marker observation record loaded, ', num2str(this.mk.num), ' records saved.']);
disp(' ');

%% reading odometry record data
disp(['Reading odometry record data from "', this.InputOdoFilePath, '"...']);
odoFileId = fopen(this.InputOdoFilePath, 'r');
this.odo = struct('lp',[], 'x',[],'y',[],'theta',[], 'num', []);
this.time = struct('lp',[],'t_odo',[],'t_mk',[]);

% read loop
while ~feof(odoFileId)
    lineTmp = fgetl(odoFileId);
    if lineTmp(1) == '#'
        continue;
    end
    lineTmp = str2num(lineTmp);
    
    % read odometry measurements
    this.odo.lp = [this.odo.lp; lineTmp(1)];
    this.odo.x = [this.odo.x; lineTmp(4)];
    this.odo.y = [this.odo.y; lineTmp(5)];
    this.odo.theta = [this.odo.theta; lineTmp(6)];
    
    % read time offset measurements
    this.time.lp = [this.time.lp; lineTmp(1)];
    this.time.t_odo = [this.time.t_odo; lineTmp(2)];
    this.time.t_mk = [this.time.t_mk; lineTmp(3)];    
    
    %     if (mod(lineTmp(1),100) == 0)
    %         disp(['odometry info loaded before lp: ', num2str(lineTmp(1))]);
    %     end
end
fclose(odoFileId);
this.odo.num = numel(this.odo.lp);
disp(['All odometry record loaded, ', num2str(this.odo.num), ' records saved.']);
disp(' ');

end

