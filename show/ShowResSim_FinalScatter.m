clear;
close all;

%% init

% path
PathFold = '..\data\sim-sqrmap-inout-20160118\';
PathFoldRes = [PathFold, 'res\'];

OdoNs = 3; 
MkNs = 3;
InitNum = 50;
SetNum = 50;

NameMkStr = ['Mk-z', num2str(MkNs), '-xy', num2str(MkNs)];
NameOdoStr = ['Odo-l', num2str(OdoNs), '-r', num2str(OdoNs)];

% flags
FlagDrawInit = 1;
FlagDrawSet = 1;

% plot configuration
FigPos = [1,1,360,360];

% referece
x_ref = [0;0;1/sqrt(2);-1/sqrt(2);0;0];

% configuration
recmat_x = cell(6,1);
recmat_x2 = cell(6,1);
recmat_dt = [];

% output prefix

if FlagDrawInit && FlagDrawSet
    strOutputPrefix = ['.\temp\', 'Mk-', num2str(MkNs), '-Odo-', num2str(OdoNs), '-s-i-'];
elseif FlagDrawSet
    strOutputPrefix = ['.\temp\', 'Mk-', num2str(MkNs), '-Odo-', num2str(OdoNs), '-s-'];
elseif FlagDrawInit
    strOutputPrefix = ['.\temp\', 'Mk-', num2str(MkNs), '-Odo-', num2str(OdoNs), '-i-'];
end

%% read data

if FlagDrawSet    
    InitId = 1;    
    for i = 1:SetNum
        SetId = i;
        
        PathRecData = [PathFoldRes, 'res-', NameMkStr, '-', NameOdoStr,...
            '-s', num2str(SetId), '-i', num2str(InitId), '.mat'];
        
        load(PathRecData);
        
        recmat_x{1} = [recmat_x{1} rec_x(:,1)];
        recmat_x{2} = [recmat_x{2} rec_x(:,2)];
        recmat_x{3} = [recmat_x{3} rec_x(:,3)];
        recmat_x{4} = [recmat_x{4} rec_x(:,4)];
        recmat_x{5} = [recmat_x{5} rec_x(:,5)];
        recmat_x{6} = [recmat_x{6} rec_x(:,6)];
    end
    
end

% read data from same set but different init
if FlagDrawInit    
    SetId = 1;    
    for i = 1:InitNum
        
        InitId = i;
        
        NameMkStr = ['Mk-z', num2str(MkNs), '-xy', num2str(MkNs), ...
            ];
        NameOdoStr = ['Odo-l', num2str(OdoNs), '-r', num2str(OdoNs), ...
            ];
        PathRecData = [PathFoldRes, 'res-',NameMkStr, '-', NameOdoStr,...
            '-s', num2str(SetId), '-i', num2str(InitId), '.mat'];
        
        load(PathRecData);
        
        recmat_x2{1} = [recmat_x2{1} rec_x(:,1)];
        recmat_x2{2} = [recmat_x2{2} rec_x(:,2)];
        recmat_x2{3} = [recmat_x2{3} rec_x(:,3)];
        recmat_x2{4} = [recmat_x2{4} rec_x(:,4)];
        recmat_x2{5} = [recmat_x2{5} rec_x(:,5)];
        recmat_x2{6} = [recmat_x2{6} rec_x(:,6)];
    end
end

%% plot final results dx-dy
fig = figure; hold on; axis equal; grid on; box on;

fig.Position = FigPos;

if FlagDrawSet
    plot(recmat_x{5}(end,:)-x_ref(5), recmat_x{6}(end,:)-x_ref(6), 'o', 'Color', 'b');
end
if FlagDrawInit
    plot(recmat_x2{5}(end,:)-x_ref(5), recmat_x2{6}(end,:)-x_ref(6), '+', 'Color', 'r');
end

set(gca, 'xlim', [-100 100]);
set(gca, 'ylim', [-100 100]);
set(gca, 'XTick', -1000:50:1000);
set(gca, 'YTick', -1000:50:1000);

set(gcf, 'Position', FigPos);
xlabel('Err. x (mm)');
ylabel('Err. y (mm)');
title(['Final Result:', ' MkNs-', num2str(MkNs), '% OdoNs-', num2str(OdoNs), '%']...
    ,'FontWeight','bold');
set(gcf, 'PaperPositionMode', 'auto');

print([strOutputPrefix, 'dx-dy-final'], '-depsc', '-r0');
print([strOutputPrefix, 'dx-dy-final'], '-dmeta', '-r0');
print([strOutputPrefix, 'dx-dy-final'], '-djpeg', '-r0');


%% plot final results q0-q1
fig = figure; hold on; axis equal; grid on; box on;

fig.Position = FigPos;

if FlagDrawSet
    plot(recmat_x{1}(end,:)-x_ref(1), recmat_x{2}(end,:)-x_ref(2), 'o', 'Color', 'b');
end
if FlagDrawInit
    plot(recmat_x2{1}(end,:)-x_ref(1), recmat_x2{2}(end,:)-x_ref(2), '+', 'Color', 'r');
end

set(gca, 'xlim', [-0.02 0.02]);
set(gca, 'ylim', [-0.02 0.02]);
set(gca, 'XTick', -1:0.01:1);
set(gca, 'YTick', -1:0.01:1);

set(gcf, 'Position', FigPos);
xlabel('Err. q0 (rad)');
ylabel('Err. q1 (rad)');
title(['Final Result:', ' MkNs-', num2str(MkNs), '% OdoNs-', num2str(OdoNs), '%']...
    ,'FontWeight','bold');
set(gcf, 'PaperPositionMode', 'auto');

print([strOutputPrefix, 'dq0-dq1-final'], '-depsc', '-r0');
print([strOutputPrefix, 'dq0-dq1-final'], '-dmeta', '-r0');
print([strOutputPrefix, 'dq0-dq1-final'], '-djpeg', '-r0');


%% plot final results q2-q3
fig = figure; hold on; axis equal; grid on; box on;

fig.Position = FigPos;

if FlagDrawSet
    plot(recmat_x{3}(end,:)-x_ref(3), recmat_x{4}(end,:)-x_ref(4), 'o', 'Color', 'b');
end
if FlagDrawInit
    plot(recmat_x2{3}(end,:)-x_ref(3), recmat_x2{4}(end,:)-x_ref(4), '+', 'Color', 'r');
end

set(gca, 'xlim', [-0.02 0.02]);
set(gca, 'ylim', [-0.02 0.02]);
set(gca, 'XTick', -1:0.01:1);
set(gca, 'YTick', -1:0.01:1);

set(gcf, 'Position', FigPos);
xlabel('Err. q2 (rad)');
ylabel('Err. q3 (rad)');
title(['Final Result:', ' MkNs-', num2str(MkNs), '% OdoNs-', num2str(OdoNs), '%']...
    ,'FontWeight','bold');
set(gcf, 'PaperPositionMode', 'auto');

print([strOutputPrefix, 'dq2-dq3-final'], '-depsc', '-r0');
print([strOutputPrefix, 'dq2-dq3-final'], '-dmeta', '-r0');
print([strOutputPrefix, 'dq2-dq3-final'], '-djpeg', '-r0');


