function DrawErr_Slam( cellMat_errMk, cellMat_errOdo, cellMat_mkNorm, cellMat_odoNorm )
%DRAWERR_SLAM Summary of this function goes here
%   Detailed explanation goes here

% set(gca,'Xtick',-10000:2000:10000) 
% set(gca,'Ytick',-10000:2000:10000) 
% title(strTitle, 'FontWeight','bold');
% xlabel('X (mm)');
% ylabel('Y (mm)');
% view(2);
% plot3(err_Mk(:,1), err_Mk(:,2), err_Mk(:,3), '.', 'Color', 'r');

%% Draw errMk

text.strTitle = 'Error: Mark Observation XY';
text.strXLabel = 'X (mm)';
text.strYLabel = 'Y (mm)';
vec_idx = [1 2];
PlotErr( cellMat_errMk, vec_idx, text );
set(gcf, 'PaperPositionMode', 'auto');
fileNameFigOutput = '.\temp\mkErr-xy';
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');

text.strTitle = 'Error: Mark Observation YZ';
text.strXLabel = 'Y (mm)';
text.strYLabel = 'Z (mm)';
vec_idx = [2 3];
PlotErr( cellMat_errMk, vec_idx, text );
set(gcf, 'PaperPositionMode', 'auto');
fileNameFigOutput = '.\temp\mkErr-yz';
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');

text.strTitle = 'Error: Mark Observation ZX';
text.strXLabel = 'Z (mm)';
text.strYLabel = 'X (mm)';
vec_idx = [3 1];
PlotErr( cellMat_errMk, vec_idx, text );
set(gcf, 'PaperPositionMode', 'auto');
fileNameFigOutput = '.\temp\mkErr-zx';
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');

%% draw errMkNorm

text.strTitle = 'Normalized Error: Mark Observation XY';
text.strXLabel = 'X (mm)';
text.strYLabel = 'Y (mm)';
vec_idx = [1 2];
PlotErr( cellMat_mkNorm, vec_idx, text );
set(gcf, 'PaperPositionMode', 'auto');
fileNameFigOutput = '.\temp\mkErrNorm-xy';
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');

text.strTitle = 'Normalized Error: Mark Observation YZ';
text.strXLabel = 'Y (mm)';
text.strYLabel = 'Z (mm)';
vec_idx = [2 3];
PlotErr( cellMat_mkNorm, vec_idx, text );
set(gcf, 'PaperPositionMode', 'auto');
fileNameFigOutput = '.\temp\mkErrNorm-yz';
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');

text.strTitle = 'Normalized Error: Mark Observation ZX';
text.strXLabel = 'Z (mm)';
text.strYLabel = 'X (mm)';
vec_idx = [3 1];
PlotErr( cellMat_mkNorm, vec_idx, text );
set(gcf, 'PaperPositionMode', 'auto');
fileNameFigOutput = '.\temp\mkErrNorm-zx';
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');

%% draw errOdo

text.strTitle = 'Error: Odometry XY';
text.strXLabel = 'X (mm)';
text.strYLabel = 'Y (mm)';
vec_idx = [1 2];
PlotErr( cellMat_errOdo, vec_idx, text );
set(gcf, 'PaperPositionMode', 'auto');
fileNameFigOutput = '.\temp\odoErr-xy';
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');

text.strTitle = 'Error: Odometry YTheta';
text.strXLabel = 'Y (mm)';
text.strYLabel = 'Theta (rad)';
vec_idx = [2 3];
PlotErr( cellMat_errOdo, vec_idx, text, false );
set(gcf, 'PaperPositionMode', 'auto');
fileNameFigOutput = '.\temp\odoErr-yt';
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');

text.strTitle = 'Error: Odometry XTheta';
text.strXLabel = 'X (mm)';
text.strYLabel = 'Theta (rad)';
vec_idx = [1 3];
PlotErr( cellMat_errOdo, vec_idx, text, false );
set(gcf, 'PaperPositionMode', 'auto');
fileNameFigOutput = '.\temp\odoErr-xt';
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');

%% draw errOdoNorm

text.strTitle = 'Normalized Error: Odometry XY';
text.strXLabel = 'X (mm)';
text.strYLabel = 'Y (mm)';
vec_idx = [1 2];
PlotErr( cellMat_odoNorm, vec_idx, text );
set(gcf, 'PaperPositionMode', 'auto');
fileNameFigOutput = '.\temp\odoErr-xy';
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');

text.strTitle = 'Normalized Error: Odometry YTheta';
text.strXLabel = 'Y (mm)';
text.strYLabel = 'Theta (rad)';
vec_idx = [2 3];
PlotErr( cellMat_odoNorm, vec_idx, text, false );
set(gcf, 'PaperPositionMode', 'auto');
fileNameFigOutput = '.\temp\odoErr-yt';
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');

text.strTitle = 'Normalized Error: Odometry XTheta';
text.strXLabel = 'X (mm)';
text.strYLabel = 'Theta (rad)';
vec_idx = [1 3];
PlotErr( cellMat_odoNorm, vec_idx, text, false );
set(gcf, 'PaperPositionMode', 'auto');
fileNameFigOutput = '.\temp\odoErr-xt';
print(fileNameFigOutput, '-depsc', '-r0');
print(fileNameFigOutput, '-dmeta', '-r0');
print(fileNameFigOutput, '-djpeg', '-r0');


end

