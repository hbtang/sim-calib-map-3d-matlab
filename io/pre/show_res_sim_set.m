clear;

%% init
PathFold = '..\data\sim-sqrmap-inout-20160118\res\';

OdoNs = 1; MkNs = 1; SetId = 1; InitId = 1;

rec_x_init = [];
rec_x_q1 = [];
rec_x_q2 = [];
rec_x_q3 = [];
rec_x_final = [];
recmat_x = cell(6,1);
recmat_dt = [];

FigPos = [1,1,480,360];

%% read data
numCalibResSet = 20;

for i = 1:numCalibResSet
    
    SetId = i;
%     InitId = i;
                
    NameMkStr = ['Mk-z', num2str(MkNs), '-xy', num2str(MkNs), ...
                    ];
    NameOdoStr = ['Odo-l', num2str(OdoNs), '-r', num2str(OdoNs), ...
                    ];            
    PathRecData = [PathFold, '/res-',NameMkStr, '-', NameOdoStr,...
        '-s', num2str(SetId), '-i', num2str(InitId), '.mat'];
    
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

x_ref = [0;0;1/sqrt(2);-1/sqrt(2);0;0];

%% boxplot
for i = 1:6
    
    f = figure; hold on; grid on;
    set(gcf, 'Position', FigPos);
    
    boxplot(recmat_x{i}(1:30:end,:).'-x_ref(i), floor(rec_lp(1:30:end)/30)); 
    
    set(gca,'FontSize',10);
    set(findobj(gca,'Type','text'),'FontSize',10, 'Rotation',45);
    
    t_xlable = xlabel('Time(sec)');
    xlabh = get(gca,'XLabel');
    set(xlabh, 'Position', get(xlabh,'Position') - [0 5 0]);
    set(xlabh, 'Rotation', 0);
    
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

%% draw single result
NameMkStr = ['Mk-z', num2str(MkNs), '-xy', num2str(MkNs), ...
                    ];
NameOdoStr = ['Odo-l', num2str(OdoNs), '-r', num2str(OdoNs), ...
                    ];            
PathRecData = [PathFold, '/res-',NameMkStr, '-', NameOdoStr,...
        '-s', num2str(1), '-i', num2str(1), '.mat'];

load(PathRecData);
rec_err = [];
sz = size(rec_x);

for i = 1:sz(1)
    rec_err = [rec_err; rec_x(i,:) - x_ref.'];
end

% rotation part
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

% translation part
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

%% plot final results
figure; hold on; axis equal; grid on;
plot(recmat_x{5}(end,:), recmat_x{6}(end,:), '.');

% set(gca, 'xlim', [-10000 5000]);
% set(gca, 'ylim', [-3000 8000]);
% set(gca, 'XTick', -10000:2000:10000);
% set(gca, 'YTick', -10000:2000:10000);

set(gcf, 'Position', FigPos);
xlabel('X (mm)');
ylabel('Y (mm)');
title('Online Extrinsic Calibration: Final Result','FontWeight','bold');
set(gcf, 'PaperPositionMode', 'auto');
print('./temp/allsets-end-xy','-depsc','-r0');

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

