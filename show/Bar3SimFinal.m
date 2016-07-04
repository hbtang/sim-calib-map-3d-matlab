function [ fh ] = Bar3SimFinal( mat_data, fileNameFigOutput, strZLabel, strTitle, FigPos)

% show mat_numOutlier_s
fh = figure;
bh = bar3(mat_data(2:6, 2:6));
az = -135; el = 30; view(az,el);
ax = gca;
ax.XLim = [0 6]; ax.YLim = [0 6];

for i = 1:5
%     ax.XTickLabel{i} = num2str(i-1);
%     ax.YTickLabel{i} = num2str(i-1);
    ax.XTickLabel{i} = num2str(i);
    ax.YTickLabel{i} = num2str(i);
end

for k = 1:length(bh)
    zdata = bh(k).ZData;
    bh(k).CData = zdata;
    bh(k).FaceColor = 'interp';
end

set(gcf, 'Position', FigPos);

xlabel('OdoNs (%)'); ylabel('MkNs (%)'); 
% zlabel('Rot. RMSE (rad)');
zlabel(strZLabel);
% title(['Final Result: ', 'Rot. RMSE'] ,'FontWeight','bold');
title(strTitle ,'FontWeight','bold');

set(gcf, 'PaperPositionMode', 'auto');
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');

end