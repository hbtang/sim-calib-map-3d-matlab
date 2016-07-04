function AddNoise2Rec(this, noiseConfig, foldPath, setId)
%ADDNOISE2REC impose random noise into current rec file

%% reading odometry record data
disp(['Reading odometry record data...']);

if nargin < 3
    foldPath = '../data/sim/';
end

if nargin < 4
    setId = 1;
end
    
    
OutputFileOdoId = fopen( [foldPath,'rec/Odo.rec'], 'r');
odo = struct('lp',[], 'x',[],'y',[],'theta',[], 'num', []);

while ~feof(OutputFileOdoId)
    lineTmp = fgetl(OutputFileOdoId);
    if lineTmp(1) == '#'
        continue;
    end
    lineTmp = str2num(lineTmp);
    
    odo.lp = [odo.lp; lineTmp(1)];
    odo.x = [odo.x; lineTmp(4)];
    odo.y = [odo.y; lineTmp(5)];
    odo.theta = [odo.theta; lineTmp(6)];
    
end
fclose(OutputFileOdoId);
odo.num = numel(odo.lp);

%% impose noise

odoNs = struct('lp',odo.lp(1), 'x',odo.x(1),'y',odo.y(1),'theta',odo.theta(1), 'num', []);
stdErrRatioOdoLin = noiseConfig.stdErrRatioOdoLin;
stdErrRatioOdoRot = noiseConfig.stdErrRatioOdoRot;

for i = 2:odo.num
    lp = odo.lp(i);
    
    ps2d_o_b1 = [odo.x(i-1);odo.y(i-1);odo.theta(i-1)];
    ps2d_o_b2 = [odo.x(i);odo.y(i);odo.theta(i)];
    ps2d_b1_b2 = FunRelPos2d(ps2d_o_b1, ps2d_o_b2);
    
%     if abs(ps2d_b1_b2(3)) > pi/6
%         db = 1;
%     end
        
    stdErrLin = stdErrRatioOdoLin*norm(ps2d_b1_b2(1:2));
    stdErrRot = stdErrRatioOdoRot*abs(ps2d_b1_b2(3));
    ps2d_b1_b2 = normrnd(ps2d_b1_b2, [stdErrLin;stdErrLin;stdErrRot]);
    
    ps2d_oNs_b1 = [odoNs.x(i-1);odoNs.y(i-1);odoNs.theta(i-1)];
    ps2d_oNs_b2 = FunMove2d(ps2d_oNs_b1, ps2d_b1_b2);
    
    odoNs.x(i) = ps2d_oNs_b2(1);
    odoNs.y(i) = ps2d_oNs_b2(2);
    odoNs.theta(i) = ps2d_oNs_b2(3);
    odoNs.lp(i) = lp;
end

%% write into record file OdoNs.rec
OutputFileOdoNoiseId = fopen([foldPath,'rec/Odo',...
    '-l',num2str(stdErrRatioOdoLin*100),'-r',num2str(stdErrRatioOdoRot*100),...
    '-s',num2str(setId), '.rec'],'w');

fprintf(OutputFileOdoNoiseId, '# odometry info\n');
fprintf(OutputFileOdoNoiseId, '# format: lp timeOdo timeCam x y theta\n');

for i = 1:numel(odoNs.lp)
    strOdo = [num2str(odoNs.lp(i)), ' '];
    strOdo = [strOdo, '0 0 '];
    strOdo = [strOdo, num2str(odoNs.x(i)), ' ',...
        num2str(odoNs.y(i)), ' ', num2str(odoNs.theta(i)), '\n',];
    fprintf(OutputFileOdoNoiseId, strOdo);
end

fclose(OutputFileOdoNoiseId);

%% reading mark record data
disp(['Reading mark record data...']);

if nargin == 2
    foldPath = './data/sim/';
end    
    
OutputFileMkId = fopen( [foldPath,'rec/Mk.rec'], 'r');
mk = struct('lp',[], 'id',[],'rvec',[],'tvec',[]);

while ~feof(OutputFileMkId)
    lineTmp = fgetl(OutputFileMkId);
    if lineTmp(1) == '#'
        continue;
    end
    lineTmp = str2num(lineTmp);
    
    mk.lp = [mk.lp; lineTmp(1)];
    mk.id = [mk.id; lineTmp(2)];
    mk.rvec = [mk.rvec; lineTmp(3:5)];
    mk.tvec = [mk.tvec; lineTmp(6:8)];
    
end
fclose(OutputFileMkId);
mk.num = numel(mk.lp);

%% impose noise

mkNs = struct('lp',[], 'id',[],'rvec',[],'tvec',[]);
stdErrRatioMkDpth = noiseConfig.stdErrRatioMkDpth;
stdErrRatioMkLtr = noiseConfig.stdErrRatioMkLtr;

for i = 1:mk.num
    lp = mk.lp(i);
    id = mk.id(i);
    tvec_c_m = mk.tvec(i,:);
    rvec_c_m = mk.rvec(i,:);
    
    stdErrDpth = stdErrRatioMkDpth*abs(tvec_c_m(3));
    stdErrLtr = stdErrRatioMkLtr*abs(tvec_c_m(3));
    
    tvec_c_m = normrnd(tvec_c_m, [stdErrLtr stdErrLtr stdErrDpth]);   

    mkNs.lp = [mkNs.lp; lp];
    mkNs.id = [mkNs.id; id];
    mkNs.rvec = [mkNs.rvec; rvec_c_m];
    mkNs.tvec = [mkNs.tvec; tvec_c_m];    
end

%% write into record file MkNs.rec
OutputFileMkNoiseId = fopen([foldPath,'rec/Mk',...
    '-z',num2str(100*stdErrRatioMkDpth),'-xy',num2str(100*stdErrRatioMkLtr),...
    '-s',num2str(setId),'.rec'],'w');

fprintf(OutputFileMkNoiseId, '# aruco mark observation info\n');
fprintf(OutputFileMkNoiseId, '# format: lp id rvec(x y z) tvec(x y z) ptimg(x1 y1 x2 y2 x3 y3 x4 y4)\n');

for i = 1:numel(mkNs.lp)
    strMk1 = [num2str(mkNs.lp(i)), ' '];
    strMk2 = [strMk1, num2str(mkNs.id(i)), ' '];
    strMk = [strMk2, num2str(mkNs.rvec(i,1)), ' ', num2str(mkNs.rvec(i,2)), ' ', num2str(mkNs.rvec(i,3)), ' ',...
        num2str(mkNs.tvec(i,1)), ' ', num2str(mkNs.tvec(i,2)), ' ', num2str(mkNs.tvec(i,3)), '\n',];
    fprintf(OutputFileMkNoiseId, strMk);
end
fclose(OutputFileMkNoiseId);

end

