function init( this )
%INIT init function, load map and calib info, set output path, ...
if nargin == 2
    simFolderPath = inputPath;
else
    simFolderPath = './data/sim/';
end

this.InputFileMapId = fopen([simFolderPath, 'cfg/Map.cfg'],'r');
this.InputFileCamId = fopen([simFolderPath, 'cfg/Cam.cfg'],'r');
this.OutputFileOdoId = fopen([simFolderPath, 'rec/Odo.rec'],'w+');
this.OutputFileMkId = fopen([simFolderPath, 'rec/Mk.rec'],'w+');

%% print comment in record file
fprintf(this.OutputFileMkId, '# aruco mark observation info\n');
fprintf(this.OutputFileMkId, '# format: lp id rvec(x y z) tvec(x y z) ptimg(x1 y1 x2 y2 x3 y3 x4 y4)\n');
fprintf(this.OutputFileOdoId, '# odometry info\n');
fprintf(this.OutputFileOdoId, '# format: lp timeOdo timeCam x y theta\n');

%% reading map data
disp(['Reading map data...']);
mapFileId = this.InputFileMapId;

% read loop
while ~feof(mapFileId)
    lineTmp = fgetl(mapFileId);
    if lineTmp(1) == '#'
        continue;
    end
    lineTmp = str2num(lineTmp);
    
    this.map.mks.id = [this.map.mks.id; lineTmp(1)];
    this.map.mks.rvec_w_m = [this.map.mks.rvec_w_m; lineTmp(2:4)];
    this.map.mks.tvec_w_m = [this.map.mks.tvec_w_m; lineTmp(5:7)];
    
end
fclose(mapFileId);

disp(['All map data loaded.']);
disp(' ');

%% reading config data
disp(['Reading camera config...']);
camFileId = this.InputFileCamId;

% read loop
while ~feof(camFileId)
    lineTmp = fgetl(camFileId);
    if lineTmp(1) == '#'
        continue;
    end
    lineTmp = str2num(lineTmp);
    
    this.calib.rvec_b_c = lineTmp(1:3).';
    this.calib.tvec_b_c = lineTmp(4:6).';
    this.config.angleOfViewH = lineTmp(7);
    this.config.angleOfViewV = lineTmp(8);
    this.config.distRangeMin = lineTmp(9);
    this.config.distRangeMax = lineTmp(10);
end
fclose(camFileId);

this.calib.RefreshByVecbc;
disp(['All camera config data loaded.']);
disp(' ');

end

