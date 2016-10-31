function DrawMapWithMeasure( this, measure, calib, bDrawMeasure, strTitle )
%DRAWMAPWITHMEASURE Draw current map with mark observation
%   Detailed explanation goes here

switch nargin
    case 3
        strTitle = 'Mapping Results';
        bDrawMeasure = false;
    case 4
        strTitle = 'Mapping Results';       
    otherwise        
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
title(strTitle, 'FontWeight','bold');
xlabel('X (mm)');
ylabel('Y (mm)');
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
    
    if bDrawMeasure
        
        plot3(tvec_w_m_measure(1), tvec_w_m_measure(2), tvec_w_m_measure(3),...
            '.', 'Color', 'r');
        plot3([tvec_w_b(1);tvec_w_m_measure(1)], [tvec_w_b(2);tvec_w_m_measure(2)], ...
            [tvec_w_b(3);tvec_w_m_measure(3)], '-', 'Color','r');
        
    end
end

%% draw marks
plot3(this.mks.tvec_w_m(:,1), this.mks.tvec_w_m(:,2), this.mks.tvec_w_m(:,3), ...
    'o', 'LineWidth', 1.5, 'MarkerEdgeColor',[0;0;0], 'MarkerFaceColor',[1;1;1], 'MarkerSize', 10);

end


