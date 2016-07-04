clear;

%% init
% PathFold = '..\data\sim-sqrmap-inout-20160118\res\';
PathFold = '..\data\r2-rightback-bidir-mk127-2016031520\res\';
numCalibResSet = 20;

OdoNs = 1; MkNs = 1;

rec_x_init = [];
rec_x_q1 = [];
rec_x_q2 = [];
rec_x_q3 = [];
rec_x_final = [];
recmat_x = cell(6,1);
recmat_dt = [];

FigPos = [1,1,480,360];

%% read data
for i = 1:numCalibResSet
%     PathRecFold = ['Mk-z', num2str(MkNs), '-xy', num2str(MkNs), ...
%         '-Odo-l', num2str(OdoNs), '-r', num2str(OdoNs), '-s', num2str(i), '\'];
%     PathRecData = [PathFold, PathRecFold, 'rec_calib_rt_', num2str(1), '.mat'];

    PathRecData = [PathFold, 'rec_calib_rt_', num2str(i), '.mat'];
    load(PathRecData);
    
    rec_x_init = [rec_x_init; rec_x(1,:)];
    rec_x_q1 = [rec_x_q1; rec_x(50,:)];
    rec_x_q2 = [rec_x_q2; rec_x(150,:)];
    rec_x_q3 = [rec_x_q3; rec_x(300,:)];
    rec_x_final = [rec_x_final; rec_x(end,:)];
    
    recmat_x{1} = [recmat_x{1} rec_x(:,1)];
    recmat_x{2} = [recmat_x{2} rec_x(:,2)];
    recmat_x{3} = [recmat_x{3} rec_x(:,3)];
    recmat_x{4} = [recmat_x{4} rec_x(:,4)];
    recmat_x{5} = [recmat_x{5} rec_x(:,5)];
    recmat_x{6} = [recmat_x{6} rec_x(:,6)];
    
    recmat_dt = [recmat_dt rec_dt];
end

% x_ref = [0.1499; 0.1490; 0.6803; -0.7018; 175.5707; -289.4186];
x_ref = [0;0;1/sqrt(2);-1/sqrt(2);0;0];

%% draw quaternion
% figure; hold on; grid on;
% set(gcf, 'Position', FigPos);
% 
% p_init = plot(rec_x_init(:,1:4), '.', 'MarkerSize', 10);
% plot(rec_x_q1(:,1:4), ':');
% plot(rec_x_q2(:,1:4), '--', 'LineWidth', 1);
% plot(rec_x_q3(:,1:4), '-', 'LineWidth', 2);
% plot(rec_x_final(:,1:4), '-', 'LineWidth', 3);
% p_1 = plot(rec_x_q1(:,1), ':');
% p_2 = plot(rec_x_q2(:,1), '--', 'LineWidth', 1);
% p_3 = plot(rec_x_q3(:,1), '-', 'LineWidth', 2);
% p_final = plot(rec_x_final(:,1), '-', 'LineWidth', 3);
% 
% h_legend = legend([p_init;p_1;p_2;p_3;p_final], 'q1.init', 'q2.init', 'q3.init', 'q4.init',...
%     'iter50', 'iter150', 'iter300', 'final');
% 
% set(h_legend,'FontSize',10);
% 
% set(gca,'FontSize',10);
% title('Online Extrinsic Calibration: Rotation Quaternion','FontWeight','bold');
% xlabel('Initial Guess Setting Id');
% ylabel('rad');
% 
% set(gcf, 'PaperPositionMode', 'auto');
% print('./temp/allsets-rot','-depsc','-r0');

%% draw translation
% figure;hold on;grid on;
% set(gcf, 'Position', FigPos);
% 
% % axis equal;
% % plot(rec_x_init(:,5),rec_x_init(:,6), '.', 'Color', 'b');
% % plot(rec_x_final(:,5),rec_x_final(:,6), '.', 'Color', 'r');
% p_init = plot(rec_x_init(:,5:6), '.', 'MarkerSize', 10);
% p_x_2 = plot(rec_x_q1(:,5), ':', 'Color', 'b');
% p_x_3 = plot(rec_x_q2(:,5), '--', 'LineWidth', 1);
% p_x_4 = plot(rec_x_q3(:,5), '-', 'LineWidth', 1);
% p_x_5 = plot(rec_x_final(:,5), '-', 'LineWidth', 2);
% 
% plot(rec_x_q1(:,5:6), ':');
% plot(rec_x_q2(:,5:6), '--', 'LineWidth', 1);
% plot(rec_x_q3(:,5:6), '-', 'LineWidth', 2);
% plot(rec_x_final(:,5:6), '-', 'LineWidth', 3);
% 
% h_type = legend([p_init;p_x_2;p_x_3;p_x_4;p_x_5], 'x.init', 'y.init', 'iter50', 'iter150', 'iter300', 'final',...
%     'Location', 'northeast');
% set(h_type,'Fontsize',10);
% % new_handle = copyobj(h_type, f_trans);
% % h_color = legend([p_x_1,p_y_1], 'x', 'y', 'Location', 'southeast');
% % set(h_color, 'Fontsize', 15);
% 
% set(gca,'FontSize',10);
% title('Online Extrinsic Calibration: Translation', 'FontWeight','bold');
% xlabel('Initial Guess Setting Id');
% ylabel('mm');
% 
% set(gcf, 'PaperPositionMode', 'auto');
% print('./temp/allsets-trans','-depsc','-r0');

%% boxplot
for i = 1:6
    
    f = figure; hold on; grid on;
    set(gcf, 'Position', FigPos);
    
    boxplot(recmat_x{i}(1:30:end,:).'-x_ref(i), floor(rec_lp(1:30:end)/30));
    
    set(findobj(gca,'Type','text'),'FontSize',10, 'Rotation',45);
    xlabel('Time(sec)');    
    xlabh = get(gca,'XLabel');
    set(xlabh,'Position',get(xlabh,'Position') - [0 5 0], 'Rotation', 0);    
    
    % set y label
    if i <= 4
        ylabel('Quaternion(rad)');
    elseif i == 5
        ylabel('X-Offset(mm)');
    elseif i == 6
        ylabel('Y-Offset(mm)');
    end    
    
    % to solve the bug that position of xlebel will reset when saving
    ax = findobj(gcf,'type','axes');
    for j=1:length(ax),
        boxparent = getappdata(ax(j),'boxplothandle');
        listeners = getappdata(boxparent,'boxlisteners');
        for k = 1:length(listeners),
            listeners{k}.Enabled = 'off';
        end
    end
    
    % save
    if i <= 4
        title(['Online Extrinsic Calibration: Rotation q',num2str(i)],'FontWeight','bold');
        
        set(gcf, 'PaperPositionMode', 'auto');
        print(['./temp/allsets-q', num2str(i)],'-depsc','-r0');
        
    elseif i == 5
        title('Online Extrinsic Calibration: Translation x','FontWeight','bold');
        
        set(gcf, 'PaperPositionMode', 'auto');
        print('./temp/allsets-x','-depsc','-r0');
    else
        title('Online Extrinsic Calibration: Translation y','FontWeight','bold');
        
        set(gcf, 'PaperPositionMode', 'auto');
        print('./temp/allsets-y','-depsc','-r0');
    end
    
end

%% draw computation time

figure; hold on; grid on;
set(gcf, 'Position', FigPos);
plot(mean(recmat_dt.'));
xlabel('Number of Iteration');
ylabel('Computation Time (sec)');
title('Online Extrinsic Calibration: Computation Time','FontWeight','bold');

set(gcf, 'PaperPositionMode', 'auto');
print('./temp/allsets-dt','-depsc','-r0');

%% create video rot
%
% VideoObj = VideoWriter('./temp/allset-rot.avi', 'Motion JPEG AVI' );
% VideoObj.FrameRate = 30;
% open(VideoObj);
%
% sz = size(recmat_x{1});
%
% h_figure = figure; hold on; grid on;
% set(gcf, 'Position', [1,1,640,480]);
% set(gca, 'ylim', [-1.5 1.5]);
%
% h_plot = plot([recmat_x{1}(1,:); recmat_x{2}(1,:); recmat_x{3}(1,:); recmat_x{4}(1,:)].',...
%     '.', 'MarkerSize', 10);
% h_legend = legend(h_plot, 'q1', 'q2', 'q3', 'q4');
% set(h_legend,'FontSize',10);
% set(gca,'FontSize',10);
% title('Online Extrinsic Calibration: Rotation Quaternion','FontWeight','bold');
% xlabel('Initial Guess Setting Id');
%
% for i = 1:sz(1)
%     lp_tmp = rec_lp(i);
%
%     set(h_plot(1), 'YData', recmat_x{1}(i,:).');
%     set(h_plot(2), 'YData', recmat_x{2}(i,:).');
%     set(h_plot(3), 'YData', recmat_x{3}(i,:).');
%     set(h_plot(4), 'YData', recmat_x{4}(i,:).');
%
%     title(['Online Extrinsic Calibration: Rotation Quaternion Iter ', num2str(lp_tmp)],'FontWeight','bold');
%
%     frame = getframe(h_figure);
%     writeVideo(VideoObj, frame);
%
% end
%
% close(VideoObj);

%% create video trans
% VideoObj = VideoWriter('./temp/allset-trans.avi', 'Motion JPEG AVI' );
% VideoObj.FrameRate = 30;
% open(VideoObj);
%
% sz = size(recmat_x{1});
%
% h_figure = figure; hold on; grid on;
% set(gcf, 'Position', [1,1,640,480]);
% set(gca, 'ylim', [-1000 1000]);
%
% h_plot = plot([recmat_x{5}(1,:); recmat_x{6}(1,:) ].',...
%     '.', 'MarkerSize', 10);
% h_legend = legend(h_plot, 'x', 'y');
% set(h_legend,'FontSize',10);
% set(gca,'FontSize',10);
% title('Online Extrinsic Calibration: Translation','FontWeight','bold');
% xlabel('Initial Guess Setting Id');
%
% Ylim = [-2000 2000];
% set(gca, 'ylim', Ylim);
%
% for i = 1:sz(1)
%     lp_tmp = rec_lp(i);
%
%     set(h_plot(1), 'YData', recmat_x{5}(i,:).');
%     set(h_plot(2), 'YData', recmat_x{6}(i,:).');
%
%     title(['Online Extrinsic Calibration: Rotation Quaternion Iter ', num2str(lp_tmp)],'FontWeight','bold');
%
%     frame = getframe(h_figure);
%     writeVideo(VideoObj, frame);
%
% end
%
% close(VideoObj);


%% draw single result
PathRecData = [PathFold, 'rec_calib_rt_', num2str(1), '.mat'];
load(PathRecData);
rec_err = [];
sz = size(rec_x);

for i = 1:sz(1)
    rec_err = [rec_err; rec_x(i,:) - x_ref.'];
end

%% rotation part
vec_id = [1;2;3;4];
DrawPlotEnv(rec_err, rec_sigma, vec_id, rec_lp );

set(gcf, 'Position', FigPos);
xlabel('Time (sec)');
ylabel('Rotation Quaternion(rad)');
title('Online Extrinsic Calibration: Rotation','FontWeight','bold');

legend(gca, 'q1', 'q2', 'q3', 'q4');

set(gcf, 'PaperPositionMode', 'auto');
print('./temp/singleset-rot','-depsc','-r0');

Xlim = get(gca, 'xlim');
Xlim(1) = 20;
set(gca, 'xlim', Xlim);
title('Online Extrinsic Calibration: Rotation (tail)','FontWeight','bold');

set(gcf, 'PaperPositionMode', 'auto');
print('./temp/singleset-rot-tail','-depsc','-r0');

%% translation part
vec_id = [5;6];
DrawPlotEnv(rec_err, rec_sigma, vec_id, rec_lp );

set(gcf, 'Position', FigPos);
xlabel('Time (sec)');
ylabel('Translation Offset (mm)');
title('Online Extrinsic Calibration: Translation','FontWeight','bold');

legend(gca, 'x', 'y');

set(gcf, 'PaperPositionMode', 'auto');
print('./temp/singleset-trans','-depsc','-r0');

Xlim = get(gca, 'xlim');
Xlim(1) = 20;
set(gca, 'xlim', Xlim);
title('Online Extrinsic Calibration: Translation (tail)','FontWeight','bold');

set(gcf, 'PaperPositionMode', 'auto');
print('./temp/singleset-trans-tail','-depsc','-r0');

%% video
% figure; hold on; grid on; axis equal;
% set(gcf, 'Position', [1,1,640,480]);
% set(gca,'XTick',-10000:250:10000);
% set(gca,'YTick',-10000:250:10000);
%
% axisLength = 200;
%
% % draw odometry frame
% h_x_odo = plot3([0;axisLength], [0;0], [0;0], ...
%     'Color', [0 0 1], 'LineWidth', 2);
% plot3([0;0], [0;axisLength], [0;0], ...
%     'Color', [0 1 0], 'LineWidth', 2);
% plot3([0;0], [0;0], [0;axisLength], ...
%     'Color', [1 0 0], 'LineWidth', 2);
%
%
% % draw reference frame
% q_c_b = x_ref(1:4).';
% R3_c_b = quat2dcm(q_c_b);
% pt3_c_b = [x_ref(5); x_ref(6); 0];
% T3_c_b = [R3_c_b pt3_c_b; 0 0 0 1];
%
% pt3d_fo_b = T3_c_b(1:3,4);
% pt3d_fx_b = T3_c_b(1:3,1:3)*[axisLength;0;0] + T3_c_b(1:3,4);
% pt3d_fy_b = T3_c_b(1:3,1:3)*[0;axisLength;0] + T3_c_b(1:3,4);
% pt3d_fz_b = T3_c_b(1:3,1:3)*[0;0;axisLength] + T3_c_b(1:3,4);
%
% h_x_ref = plot3([pt3d_fo_b(1);pt3d_fx_b(1)], [pt3d_fo_b(2);pt3d_fx_b(2)], [pt3d_fo_b(3);pt3d_fx_b(3)], ...
%     ':', 'Color', [0 0 1], 'LineWidth', 2);
% plot3([pt3d_fo_b(1);pt3d_fy_b(1)], [pt3d_fo_b(2);pt3d_fy_b(2)], [pt3d_fo_b(3);pt3d_fy_b(3)], ...
%     ':', 'Color', [0 1 0], 'LineWidth', 2);
% plot3([pt3d_fo_b(1);pt3d_fz_b(1)], [pt3d_fo_b(2);pt3d_fz_b(2)], [pt3d_fo_b(3);pt3d_fz_b(3)], ...
%     ':', 'Color', [1 0 0], 'LineWidth', 2);
%
% % draw current estimation
% q_c_b = rec_x(1,1:4);
% R3_c_b = quat2dcm(q_c_b);
% pt3_c_b = [rec_x(1,5); rec_x(1,6); 0];
% T3_c_b = [R3_c_b pt3_c_b; 0 0 0 1];
%
% pt3d_fo_b = T3_c_b(1:3,4);
% pt3d_fx_b = T3_c_b(1:3,1:3)*[axisLength;0;0] + T3_c_b(1:3,4);
% pt3d_fy_b = T3_c_b(1:3,1:3)*[0;axisLength;0] + T3_c_b(1:3,4);
% pt3d_fz_b = T3_c_b(1:3,1:3)*[0;0;axisLength] + T3_c_b(1:3,4);
%
% hdObjBaseX = plot3([pt3d_fo_b(1);pt3d_fx_b(1)], [pt3d_fo_b(2);pt3d_fx_b(2)], [pt3d_fo_b(3);pt3d_fx_b(3)], ...
%     'Color', [0 0 1], 'LineWidth', 3);
% hdObjBaseY = plot3([pt3d_fo_b(1);pt3d_fy_b(1)], [pt3d_fo_b(2);pt3d_fy_b(2)], [pt3d_fo_b(3);pt3d_fy_b(3)], ...
%     'Color', [0 1 0], 'LineWidth', 3);
% hdObjBaseZ = plot3([pt3d_fo_b(1);pt3d_fz_b(1)], [pt3d_fo_b(2);pt3d_fz_b(2)], [pt3d_fo_b(3);pt3d_fz_b(3)], ...
%     'Color', [1 0 0], 'LineWidth', 3);
% view(3);
%
% % legend([h_x_odo, h_x_ref, hdObjBaseX], 'odometry', 'reference', 'current')
%
% VideoObj = VideoWriter('./temp/singleset-full.avi', 'Motion JPEG AVI' );
% VideoObj.FrameRate = 30;
% open(VideoObj);
%
% % set(gca, 'xlim', [-500 500]);
% % set(gca, 'ylim', [-500 500]);
% set(gca, 'zlim', [-500 500]);
%
% az = -45; el = 0; view(az,el);
%
% strTitle = ['Online Extrinsic Calibration: Iteration ', num2str(1)];
% title(strTitle, 'FontWeight','bold');
%
% for i = 1:120
%     az = az + 3;
%     el = el + 0.5;
%     view(az,el);
%     frame = getframe(gcf);
%     writeVideo(VideoObj, frame);
% end
%
% for i = 1:sz(1)
%     q_c_b = rec_x(i,1:4);
%     R3_c_b = quat2dcm(q_c_b);
%     pt3_c_b = [rec_x(i,5); rec_x(i,6); 0];
%     T3_c_b = [R3_c_b pt3_c_b; 0 0 0 1];
%
%     pt3d_fo_b = T3_c_b(1:3,4);
%     pt3d_fx_b = T3_c_b(1:3,1:3)*[axisLength;0;0] + T3_c_b(1:3,4);
%     pt3d_fy_b = T3_c_b(1:3,1:3)*[0;axisLength;0] + T3_c_b(1:3,4);
%     pt3d_fz_b = T3_c_b(1:3,1:3)*[0;0;axisLength] + T3_c_b(1:3,4);
%
%     set(hdObjBaseX, 'XData', [pt3d_fo_b(1);pt3d_fx_b(1)]);
%     set(hdObjBaseX, 'YData', [pt3d_fo_b(2);pt3d_fx_b(2)]);
%     set(hdObjBaseX, 'ZData', [pt3d_fo_b(3);pt3d_fx_b(3)]);
%
%     set(hdObjBaseY, 'XData', [pt3d_fo_b(1);pt3d_fy_b(1)]);
%     set(hdObjBaseY, 'YData', [pt3d_fo_b(2);pt3d_fy_b(2)]);
%     set(hdObjBaseY, 'ZData', [pt3d_fo_b(3);pt3d_fy_b(3)]);
%
%     set(hdObjBaseZ, 'XData', [pt3d_fo_b(1);pt3d_fz_b(1)]);
%     set(hdObjBaseZ, 'YData', [pt3d_fo_b(2);pt3d_fz_b(2)]);
%     set(hdObjBaseZ, 'ZData', [pt3d_fo_b(3);pt3d_fz_b(3)]);
%
%     strTitle = ['Online Extrinsic Calibration: Iteration ', num2str(i),10];
%     strTitle = [strTitle, 'Rotation Error: '];
%     strTitle = [strTitle, 'dq1=', num2str(rec_err(i,1)), ' '];
%     strTitle = [strTitle, 'dq2=', num2str(rec_err(i,2)), ' '];
%     strTitle = [strTitle, 'dq3=', num2str(rec_err(i,3)), ' '];
%     strTitle = [strTitle, 'dq4=', num2str(rec_err(i,4)), ' ', 10];
%     strTitle = [strTitle, 'Translation Error: '];
%     strTitle = [strTitle, 'dx=', num2str(rec_err(i,5)), ' '];
%     strTitle = [strTitle, 'dy=', num2str(rec_err(i,6)), ' '];
%     title(strTitle, 'FontWeight','bold');
%
%     Xlim = get(gca, 'xlim');
%     Xlim(1) = min(-500,pt3d_fo_b(1)-100); Xlim(2) = max(500,pt3d_fo_b(1)+100);
%     set(gca, 'xlim', Xlim);
%     Ylim = get(gca, 'ylim');
%     Ylim(1) = min(-500,pt3d_fo_b(2)-100); Ylim(2) = max(500,pt3d_fo_b(2)+100);
%     set(gca, 'ylim', Ylim);
%     set(gca, 'zlim', [-500 500]);
%
%     frame = getframe(gcf);
%     writeVideo(VideoObj, frame);
% end
%
% for i = 1:120
%     az = az + 3;
%     el = el - 0.5;
%     view(az,el);
%     frame = getframe(gcf);
%     writeVideo(VideoObj, frame);
% end
%
% close(VideoObj);

%% display standard diviation

err_std = zeros(6,1);
err_mean = zeros(6,1);
for i = 1:6
    err_std(i) = std(recmat_x{i}(end,:));
    err_mean(i) = mean(recmat_x{i}(end,:))-x_ref(i);
end

OutputFile = fopen('./temp/evaluation.txt','w');

fprintf(OutputFile, '# calibration results: online constraint filtering\n');
fprintf(OutputFile, '# format: q1 q2 q3 q4 x y\n');
fprintf(OutputFile, 'mean of error: ');
for i = 1:6
    fprintf(OutputFile, [num2str(err_mean(i)), ' ']);
end
fprintf(OutputFile, '\n');
fprintf(OutputFile, 'std of error: ');
for i = 1:6
    fprintf(OutputFile, [num2str(err_std(i)), ' ']);
end
fprintf(OutputFile, '\n');

fclose(OutputFile);

