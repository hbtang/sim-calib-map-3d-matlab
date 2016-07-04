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
    
    % debug
    % for rec_p1_right_clock_mk120_2016031509
%     id_vec_ref = [62;71;67;64;69;49;53;55;51;47;43;57;45;41;59];
    % for rec_p1_right_anticlock_mk120_2016031422
%     id_vec_ref = [0;1;2;3;4;5;6;8;15;16;17;18;19;20;21;22;24;25;26;27;28;29;30;31;32;33;34;35;36;37;38;39;40;42;44;46;48;50];
%     if  isempty(find(id_vec_ref == lineTmp(2),1))
%         continue;
%     end
%     if lineTmp(1) < 6000
%         continue;
%     end
    % debug end    
    
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

% read loop
while ~feof(odoFileId)
    lineTmp = fgetl(odoFileId);
    if lineTmp(1) == '#'
        continue;
    end
    lineTmp = str2num(lineTmp);
    
    this.odo.lp = [this.odo.lp; lineTmp(1)];
    this.odo.x = [this.odo.x; lineTmp(4)];
    this.odo.y = [this.odo.y; lineTmp(5)];
    this.odo.theta = [this.odo.theta; lineTmp(6)];
    %     if (mod(lineTmp(1),100) == 0)
    %         disp(['odometry info loaded before lp: ', num2str(lineTmp(1))]);
    %     end
end
fclose(odoFileId);
this.odo.num = numel(this.odo.lp);
disp(['All odometry record loaded, ', num2str(this.odo.num), ' records saved.']);
disp(' ');

end

