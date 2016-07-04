%% init
% clear;
% 
% % read measurement
% PathFold = '../data/r2-rightback-bidir-mk127-2016031520';
% NameMk = 'Mk.rec'; NameOdo = 'Odo.rec';
% measure = ClassMeasure(PathFold, NameMk, NameOdo);
% measure.ReadRecData;
% measure.PruneData(200, 4*pi/180);
% 
% % set state
vec_x = [0.1499; 0.1490; 0.6803; -0.7018; 175.5707; -289.4186];
% vec_x = [0.1502; 0.1469; 0.6803; -0.7021; 194.4376; -281.0775];

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
        cnstr_tmp.pt3_m_c1 = mk.tvec(rowMk1,:).';        
        rowOdo1 = find(odo.lp == cnstr_tmp.lp1, 1);
        cnstr_tmp.ps2_b1_w = [odo.x(rowOdo1); odo.y(rowOdo1); odo.theta(rowOdo1)];
        
        cnstr_tmp.lp2 = mk.lp(rowMk2);
        cnstr_tmp.pt3_m_c2 = mk.tvec(rowMk2,:).';        
        rowOdo2 = find(odo.lp == cnstr_tmp.lp2, 1);
        cnstr_tmp.ps2_b2_w = [odo.x(rowOdo2); odo.y(rowOdo2); odo.theta(rowOdo2)];
        
        if cnstr_tmp.lp2 - cnstr_tmp.lp1 > 300
            continue;
        end        
        
        cell_cnstr{numCnstr+1} = cnstr_tmp;
        numCnstr = numCnstr+1;    
    end
end

%% error re-projection

R3_c_b = quat2rot(vec_x(1:4));
t3_c_b = [vec_x(5:6);0];
T3_c_b = [R3_c_b t3_c_b; 0 0 0 1];

mat_err = zeros(numCnstr, 3);
for i = 1:numCnstr
    cnstr_tmp = cell_cnstr{i};
    ps2_b1_w = cnstr_tmp.ps2_b1_w;
    ps2_b2_w = cnstr_tmp.ps2_b2_w;   
    
    T_b1_w = FunPs2d2T3d( ps2_b1_w );
    T_b2_w = FunPs2d2T3d( ps2_b2_w );
    
    T_b2_b1 = inv(T_b1_w)*T_b2_w;
    
    pt3_m_c1 = cnstr_tmp.pt3_m_c1;
    pt3_m_c2 = cnstr_tmp.pt3_m_c2;
    
    pt3_m_b1_1 = R3_c_b*pt3_m_c1 + t3_c_b;
    
    T3_c2_b1 = T_b2_b1*T3_c_b;
    R3_c2_b1 = T3_c2_b1(1:3,1:3);
    t3_c2_b1 = T3_c2_b1(1:3,4);    
    pt3_m_b1_2 = R3_c2_b1*pt3_m_c2 + t3_c2_b1;
    
    d_pt3_tmp = pt3_m_b1_1 - pt3_m_b1_2;
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

title('Error Re-Projection: AGV Dataset', 'FontWeight', 'bold');
set(gcf, 'PaperPositionMode', 'auto');

print('.\temp\ErrProj-xy-vmcif', '-depsc', '-r0');
print('.\temp\ErrProj-xy-vmcif', '-dmeta', '-r0');
print('.\temp\ErrProj-xy-vmcif', '-djpeg', '-r0');

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

title('Error Re-Projection: AGV Dataset', 'FontWeight', 'bold');
set(gcf, 'PaperPositionMode', 'auto');

print('.\temp\ErrProj-yz-vmcif', '-depsc', '-r0');
print('.\temp\ErrProj-yz-vmcif', '-dmeta', '-r0');
print('.\temp\ErrProj-yz-vmcif', '-djpeg', '-r0');

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

title('Error Re-Projection: AGV Dataset', 'FontWeight', 'bold');
set(gcf, 'PaperPositionMode', 'auto');

print('.\temp\ErrProj-zx-vmcif', '-depsc', '-r0');
print('.\temp\ErrProj-zx-vmcif', '-dmeta', '-r0');
print('.\temp\ErrProj-zx-vmcif', '-djpeg', '-r0');

rms(mat_err)


