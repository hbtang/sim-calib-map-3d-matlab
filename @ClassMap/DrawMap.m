function DrawMap( this, measure, calib, setting, options)
%DRAWMAPWITHMEASURE Draw current map with mark observation
%   Detailed explanation goes here

if nargin < 4
    options = [];
end
if ~isfield(options, 'bDrawMeasure')
    options.bDrawMeasure = true;
end
if ~isfield(options, 'strTitle')
    options.strTitle = 'Mapping Results';
end
if ~isfield(options, 'fileNameFigOut')
    options.fileNameFigOut = '.\temp\slam';
end
if ~isfield(options, 'bDrawMkRot')
    options.bDrawMkRot = false;
end
if ~isfield(options, 'scaleMk')
    options.scaleMk = 1;
end

%% init
% if isempty(this.hdFigMap)
%     this.hdFigMap = figure;
% end
% figure(this.hdFigMap);

figure;
set(gcf, 'Position', [800,1,640,480]);

set(gca,'Xtick',-10000:2000:10000)
set(gca,'Ytick',-10000:2000:10000)

hold on; grid on; axis equal;
title(options.strTitle, 'FontWeight','bold');
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
box on;
view(2);
% set(this.hdFigMap, 'Name', 'Current Map', 'NumberTitle', 'off');

%% draw key frames (trajectory)
plot(this.kfs.ps2d_w_b(:,1), this.kfs.ps2d_w_b(:,2), ...
    '.-', 'Color','b','MarkerSize',5);

%% draw observation
mk = measure.mk;
T3d_b_c = calib.T3d_b_c;
for i = 1:mk.num
    lp = mk.lp(i);
    lpOrdMap = find(this.kfs.lp == lp, 1);
    mkId = mk.id(i);
    mkIdOrdMap = find(this.mks.id == mkId, 1);
    
    tvec_c_m = mk.tvec(i,:).';    
    rvec_w_b = this.kfs.rvec_w_b(lpOrdMap,:).';
    tvec_w_b = this.kfs.tvec_w_b(lpOrdMap,:).';    
    T3d_w_b = FunVec2Trans3d(rvec_w_b, tvec_w_b);
    T3d_w_c = T3d_w_b*T3d_b_c;    
    tvec_w_m_measure = T3d_w_c(1:3,1:3)*tvec_c_m + T3d_w_c(1:3,4);
    
    if options.bDrawMeasure
        plot3(tvec_w_m_measure(1), tvec_w_m_measure(2), tvec_w_m_measure(3),...
            '.', 'Color', 'r');
        plot3([tvec_w_b(1);tvec_w_m_measure(1)], [tvec_w_b(2);tvec_w_m_measure(2)], ...
            [tvec_w_b(3);tvec_w_m_measure(3)], '-', 'Color','r');
    end
end

%% draw marks
tvec_m_pt1 = options.scaleMk*setting.aruco.tvec_m_pt1.';
tvec_m_pt2 = options.scaleMk*setting.aruco.tvec_m_pt2.';
tvec_m_pt3 = options.scaleMk*setting.aruco.tvec_m_pt3.';
tvec_m_pt4 = options.scaleMk*setting.aruco.tvec_m_pt4.';
if options.bDrawMkRot
    for i = 1:this.mks.numMks
        tvec_w_m = this.mks.tvec_w_m(i,:).';
        rvec_w_m = this.mks.rvec_w_m(i,:).';
        R3d_w_m = rodrigues(rvec_w_m);   
        
        tvec_w_pt1 = R3d_w_m*tvec_m_pt1 + tvec_w_m;
        tvec_w_pt2 = R3d_w_m*tvec_m_pt2 + tvec_w_m;
        tvec_w_pt3 = R3d_w_m*tvec_m_pt3 + tvec_w_m;
        tvec_w_pt4 = R3d_w_m*tvec_m_pt4 + tvec_w_m;
        mat_tvec_w_pts = [tvec_w_pt1.'; tvec_w_pt2.'; tvec_w_pt3.'; tvec_w_pt4.'];
        
        fill3(mat_tvec_w_pts(:,1), mat_tvec_w_pts(:,2), mat_tvec_w_pts(:,3), [0 0 0])
    end
else
    plot3(this.mks.tvec_w_m(:,1), this.mks.tvec_w_m(:,2), this.mks.tvec_w_m(:,3), ...
        'o', 'LineWidth', 1.5, 'MarkerEdgeColor',[0;0;0], 'MarkerFaceColor',[1;1;1], 'MarkerSize', 10);
end

%% save image
set(gcf, 'PaperPositionMode', 'auto');

view(3);
print(options.fileNameFigOut, '-depsc', '-r0');
print(options.fileNameFigOut, '-dmeta', '-r0');
print(options.fileNameFigOut, '-djpeg', '-r0');

view(0,0);
print([options.fileNameFigOut, '-xz'], '-depsc', '-r0');
print([options.fileNameFigOut, '-xz'], '-dmeta', '-r0');
print([options.fileNameFigOut, '-xz'], '-djpeg', '-r0');

view(90,0);
print([options.fileNameFigOut, '-yz'], '-depsc', '-r0');
print([options.fileNameFigOut, '-yz'], '-dmeta', '-r0');
print([options.fileNameFigOut, '-yz'], '-djpeg', '-r0');

view(2);
print([options.fileNameFigOut, '-xy'], '-depsc', '-r0');
print([options.fileNameFigOut, '-xy'], '-dmeta', '-r0');
print([options.fileNameFigOut, '-xy'], '-djpeg', '-r0');

end


