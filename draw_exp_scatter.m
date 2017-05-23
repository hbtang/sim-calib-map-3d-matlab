%% init
% clear;

% read measurement
PathFold = 'C:\Workspace\Data\cuhksz-2016.3.15\r2-rightback-bidir-mk127-2016031520';
NameMk = 'Mk.rec'; NameOdo = 'Odo.rec';
measure = ClassMeasure(PathFold, NameMk, NameOdo);
measure.ReadRecData;
measure.PruneData(200, 4*pi/180);

% set state
algorithm = 'VSLAM';
switch algorithm
    case 'VSLAM'
        % mcslam
        rvec_b_c = [-0.4281; -1.9537; 2.0169];
        tvec_b_c = [175.571; -289.419; 0];
    case 'VMCGF'
        % vmcgf
        rvec_b_c = [-0.40051; -1.9676; 2.0342];
        tvec_b_c = [133.0044; -245.3011; 0];
    case 'EKF'
        % ekf
        rvec_b_c = [-0.4240   -1.9542    2.0199].';
        tvec_b_c = [143.3491 -267.7614   0].';
end
R_b_c = rodrigues(rvec_b_c);
T_b_c = [R_b_c tvec_b_c; 0 0 0 1];



%% generate constraint record
odo = measure.odo;
mk = measure.mk;

cell_cnstr = cell(0,0);
numCnstr = 0;

for i = 1:mk.numMkId
    mkId = mk.vecMkId(i);
    vec_tmp = find(mk.id == mkId);
    
    for j = 2:numel(vec_tmp)
        rowMk1 = vec_tmp(j-1);
        rowMk2 = vec_tmp(j);
        
        cnstr_tmp.id = mkId;
        cnstr_tmp.lp1 = mk.lp(rowMk1);
        cnstr_tmp.tvec_c1_m = mk.tvec(rowMk1,:).';
        rowOdo1 = find(odo.lp == cnstr_tmp.lp1, 1);
        cnstr_tmp.se2_w_b1 = [odo.x(rowOdo1); odo.y(rowOdo1); odo.theta(rowOdo1)];
        
        cnstr_tmp.lp2 = mk.lp(rowMk2);
        cnstr_tmp.tvec_c2_m = mk.tvec(rowMk2,:).';
        rowOdo2 = find(odo.lp == cnstr_tmp.lp2, 1);
        cnstr_tmp.se2_w_b2 = [odo.x(rowOdo2); odo.y(rowOdo2); odo.theta(rowOdo2)];
        
        if cnstr_tmp.lp2 - cnstr_tmp.lp1 > 300
            continue;
        end
        
        cell_cnstr{numCnstr+1} = cnstr_tmp;
        numCnstr = numCnstr+1;
    end
end

%% error re-projection
mat_err = zeros(numCnstr, 3);
for i = 1:numCnstr
    cnstr_tmp = cell_cnstr{i};
    se2_w_b1 = cnstr_tmp.se2_w_b1;
    se2_w_b2 = cnstr_tmp.se2_w_b2;
    
    T_w_b1 = FunPs2d2T3d( se2_w_b1 );
    T_w_b2 = FunPs2d2T3d( se2_w_b2 );
    
    T_b1_b2 = inv(T_w_b1)*T_w_b2;
    
    tvec_c1_m = cnstr_tmp.tvec_c1_m;
    tvec_c2_m = cnstr_tmp.tvec_c2_m;
    
    pt3_b1_m_1 = R_b_c*tvec_c1_m + tvec_b_c;
    
    T3_b1_c2 = T_b1_b2*T_b_c;
    R3_b1_c2 = T3_b1_c2(1:3,1:3);
    tvec_b1_c2 = T3_b1_c2(1:3,4);
    pt3_b1_m_2 = R3_b1_c2*tvec_c2_m + tvec_b1_c2;
    
    d_pt3_tmp = pt3_b1_m_1 - pt3_b1_m_2;
    mat_err(i,:) = d_pt3_tmp.';
end


%% draw scatter: error re-projection, (xy, yz, zx)

FigPos = [1 1 480 360];

% draw xy
fh = figure;
hold on; grid on; box on;
axis equal;
ax = gca;
ax.XLim = [-110 110]; ax.XTick = (-1000:25:1000);
ax.YLim = [-100 100]; ax.YTick = (-1000:25:1000);
set(gcf, 'Position', FigPos);
plot(mat_err(:,1), mat_err(:,2), '.');

xlabel('Err. x (mm)');
ylabel('Err. y (mm)');

title(['Error Re-Projection: ', algorithm], 'FontWeight', 'bold');
set(gcf, 'PaperPositionMode', 'auto');

print(['.\temp\ErrProj-xy-',algorithm], '-depsc', '-r0');
print(['.\temp\ErrProj-xy-',algorithm], '-dmeta', '-r0');
print(['.\temp\ErrProj-xy-',algorithm], '-djpeg', '-r0');

% draw yz
fh = figure;
hold on; grid on; box on;
axis equal;
ax = gca;
ax.XLim = [-110 110]; ax.XTick = (-1000:25:1000);
ax.YLim = [-100 100]; ax.YTick = (-1000:25:1000);
set(gcf, 'Position', FigPos);
plot(mat_err(:,2), mat_err(:,3), '.');

xlabel('Err. y (mm)');
ylabel('Err. z (mm)');

title(['Error Re-Projection: ', algorithm], 'FontWeight', 'bold');
set(gcf, 'PaperPositionMode', 'auto');

print(['.\temp\ErrProj-yz-',algorithm], '-depsc', '-r0');
print(['.\temp\ErrProj-yz-',algorithm], '-dmeta', '-r0');
print(['.\temp\ErrProj-yz-',algorithm], '-djpeg', '-r0');

% draw zx
fh = figure;
hold on; grid on; box on;
axis equal;
ax = gca;
ax.XLim = [-110 110]; ax.XTick = (-1000:25:1000);
ax.YLim = [-100 100]; ax.YTick = (-1000:25:1000);
set(gcf, 'Position', FigPos);
plot(mat_err(:,3), mat_err(:,1), '.');

xlabel('Err. z (mm)');
ylabel('Err. x (mm)');

title(['Error Re-Projection: ', algorithm], 'FontWeight', 'bold');
set(gcf, 'PaperPositionMode', 'auto');

print(['.\temp\ErrProj-zx-',algorithm], '-depsc', '-r0');
print(['.\temp\ErrProj-zx-',algorithm], '-dmeta', '-r0');
print(['.\temp\ErrProj-zx-',algorithm], '-djpeg', '-r0');

rms(mat_err)


