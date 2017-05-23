function ReadRecData(this)
%READRECDATA read record data from Mk.rec and Odo.rec

%% reading mark record data
disp(['Reading mark record data from "', this.InputMkFilePath, '"...']);
markFileId = fopen(this.InputMkFilePath,'r');
this.mk = struct('lp', [], 'id', [], 'rvec', [], 'tvec', [], ...
    'num', [], 'numMkId', [], 'vecMkId', [], ...
    'pt1', [], 'pt2', [], 'pt3', [], 'pt4', []);

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
    
    % debug begin ...
    % select several mk
%     if lineTmp(2) > 5
%         continue;
%     end
    % debug end
        
    this.mk.lp = [this.mk.lp; lineTmp(1)];
    this.mk.id = [this.mk.id; lineTmp(2)];
    this.mk.rvec = [this.mk.rvec; lineTmp(3:5)];
    this.mk.tvec = [this.mk.tvec; lineTmp(6:8)];
    
    
    
    
    
    if numel(lineTmp) < 9
        continue;
    end    
    this.mk.pt1 = [this.mk.pt1; lineTmp(9:10)];
    this.mk.pt2 = [this.mk.pt2; lineTmp(11:12)];
    this.mk.pt3 = [this.mk.pt3; lineTmp(13:14)];
    this.mk.pt4 = [this.mk.pt4; lineTmp(15:16)];
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
    
end
fclose(odoFileId);
this.odo.num = numel(this.odo.lp);
disp(['All odometry record loaded, ', num2str(this.odo.num), ' records saved.']);
disp(' ');

end

