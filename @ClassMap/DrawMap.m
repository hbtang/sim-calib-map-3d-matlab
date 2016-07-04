function DrawMap( this )
%DRAWMAP draw current map
if isempty(this.hdFigMap)
    this.hdFigMap = figure;
end

figure(this.hdFigMap);
hold off; 
plot(this.kfs.ps2d_w_b(1,1), this.kfs.ps2d_w_b(1,2));

axis equal; grid on; hold on;
view(2);
set(this.hdFigMap, 'Name', 'Current Map', 'NumberTitle', 'off');

plot(this.kfs.ps2d_w_b(:,1), this.kfs.ps2d_w_b(:,2), ...
    '.-', 'Color','b','MarkerSize',5);
plot3(this.mks.tvec_w_m(:,1), this.mks.tvec_w_m(:,2), this.mks.tvec_w_m(:,3), ...
   'o', 'LineWidth', 2, 'MarkerEdgeColor',[0;0;0], 'MarkerFaceColor',[1 0.2 0.2], 'MarkerSize', 10);

end

