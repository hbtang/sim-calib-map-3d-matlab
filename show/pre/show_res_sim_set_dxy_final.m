clear;

%% init

% path
PathFold = '..\data\sim-sqrmap-inout-20160118\';
PathFoldRes = [PathFold, 'res\'];

OdoNs = 1; MkNs = 1;
InitId = 1; InitNum = 20;
SetId = 1; SetNum = 50;

% flags
FlagDrawInit = 1;
FlagDrawSet = 1;

% plot configuration
FigPos = [1,1,480,360];

% referece
x_ref = [0;0;1/sqrt(2);-1/sqrt(2);0;0];

for i_global = 1:1
    for j_global = 1:1
        
        MkNs = i_global;
        OdoNs = j_global;
        
        NameMkStr = ['Mk-z', num2str(MkNs), '-xy', num2str(MkNs)];
        NameOdoStr = ['Odo-l', num2str(OdoNs), '-r', num2str(OdoNs)];
        
        PathFoldFig = [PathFold, 'fig\', NameMkStr, '-', NameOdoStr, '\'];
        PathFoldFigInit = [PathFold, 'fig\', NameMkStr, '-', NameOdoStr, '-i\'];
        PathFoldFigSet = [PathFold, 'fig\', NameMkStr, '-', NameOdoStr, '-s\'];
        mkdir(PathFoldFig);
        
        % configuration
        recmat_x = cell(6,1);
        recmat_x2 = cell(6,1);
        recmat_dt = [];
        
        
        %% read data
        x_ref = [0;0;1/sqrt(2);-1/sqrt(2);0;0];
        
        if FlagDrawSet
            
            % read data of different set
            SetId = 1; InitId = 1;
            
            for i = 1:SetNum
                SetId = i;
                
                PathRecData = [PathFoldRes, 'res-',NameMkStr, '-', NameOdoStr,...
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
            
            SetId = 1; InitId = 1;
            
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
        fig = figure; hold on; axis equal; grid on;
        
        fig.Position = FigPos;
        
        if FlagDrawSet
            plot(recmat_x{5}(end,:)-x_ref(5), recmat_x{6}(end,:)-x_ref(6), 'o', 'Color', 'b');
        end
        if FlagDrawInit
            plot(recmat_x2{5}(end,:)-x_ref(5), recmat_x2{6}(end,:)-x_ref(6), '+', 'Color', 'r');
        end
        
        set(gca, 'xlim', [-200 200]);
        set(gca, 'ylim', [-200 200]);
        set(gca, 'XTick', -1000:50:1000);
        set(gca, 'YTick', -1000:50:1000);
        
        set(gcf, 'Position', FigPos);
        xlabel('Err. x (mm)');
        ylabel('Err. y (mm)');
        title(['Final Result:', ' MkNs-', num2str(MkNs), '% OdoNs-', num2str(OdoNs), '%']...
            ,'FontWeight','bold');
        set(gcf, 'PaperPositionMode', 'auto');
        
        if FlagDrawInit == 1 && FlagDrawSet == 1
            FileNameFig = [PathFoldFig, 'dxdy-final-s-i'];
        elseif FlagDrawSet == 1
            FileNameFig = [PathFoldFig, 'dxdy-final-s'];
        elseif FlagDrawInit == 1
            FileNameFig = [PathFoldFig, 'dxdy-final-i'];
        end
        
        print(FileNameFig, '-depsc', '-r0');
        print(FileNameFig, '-dmeta', '-r0');
        
        
        %% plot final results q0-q1
        fig = figure; hold on; axis equal; grid on;
        
        fig.Position = FigPos;
        
        if FlagDrawSet
            plot(recmat_x{1}(end,:)-x_ref(1), recmat_x{2}(end,:)-x_ref(2), 'o', 'Color', 'b');
        end
        if FlagDrawInit
            plot(recmat_x2{1}(end,:)-x_ref(1), recmat_x2{2}(end,:)-x_ref(2), '+', 'Color', 'r');
        end
        
        set(gca, 'xlim', [-0.04 0.04]);
        set(gca, 'ylim', [-0.04 0.04]);
        set(gca, 'XTick', -1:0.01:1);
        set(gca, 'YTick', -1:0.01:1);
        
        set(gcf, 'Position', FigPos);
        xlabel('Err. q0 (rad)');
        ylabel('Err. q1 (rad)');
        title(['Final Result:', ' MkNs-', num2str(MkNs), '% OdoNs-', num2str(OdoNs), '%']...
            ,'FontWeight','bold');
        set(gcf, 'PaperPositionMode', 'auto');
        
        if FlagDrawInit == 1 && FlagDrawSet == 1
            FileNameFig = [PathFoldFig, 'dq0dq1-final-s-i'];
        elseif FlagDrawSet == 1
            FileNameFig = [PathFoldFig, 'dq0dq1-final-s'];
        elseif FlagDrawInit == 1
            FileNameFig = [PathFoldFig, 'dq0dq1-final-i'];
        end
        
        print(FileNameFig, '-depsc', '-r0');
        print(FileNameFig, '-dmeta', '-r0');
        
        
        
        %% plot final results q2-q3
        fig = figure; hold on; axis equal; grid on;
        
        fig.Position = FigPos;
        
        if FlagDrawSet
            plot(recmat_x{3}(end,:)-x_ref(3), recmat_x{4}(end,:)-x_ref(4), 'o', 'Color', 'b');
        end
        if FlagDrawInit
            plot(recmat_x2{3}(end,:)-x_ref(3), recmat_x2{4}(end,:)-x_ref(4), '+', 'Color', 'r');
        end
        
        set(gca, 'xlim', [-0.04 0.04]);
        set(gca, 'ylim', [-0.04 0.04]);
        set(gca, 'XTick', -1:0.01:1);
        set(gca, 'YTick', -1:0.01:1);
        
        set(gcf, 'Position', FigPos);
        xlabel('Err. q2 (rad)');
        ylabel('Err. q3 (rad)');
        title(['Final Result:', ' MkNs-', num2str(MkNs), '% OdoNs-', num2str(OdoNs), '%']...
            ,'FontWeight','bold');
        set(gcf, 'PaperPositionMode', 'auto');
        
        if FlagDrawInit == 1 && FlagDrawSet == 1
            FileNameFig = [PathFoldFig, 'dq2dq3-final-s-i'];
        elseif FlagDrawSet == 1
            FileNameFig = [PathFoldFig, 'dq2dq3-final-s'];
        elseif FlagDrawInit == 1
            FileNameFig = [PathFoldFig, 'dq2dq3-final-i'];
        end
        
        print(FileNameFig, '-depsc', '-r0');
        print(FileNameFig, '-dmeta', '-r0');
        
        close all;
        
        %% display standard diviation
        %         if FlagDrawInit == 1 && FlagDrawSet == 1
        %             FileNameLog = [PathFoldFig,  'statistics-i-s.txt'];
        %         elseif FlagDrawSet == 1
        %             FileNameLog = [PathFoldFig, 'statistics-s.txt'];
        %         elseif FlagDrawInit == 1
        %             FileNameLog = [PathFoldFig, 'statistics-i.txt'];
        %         end
        %
        %         OutputFile = fopen(FileNameLog,'w');
        %         fprintf(OutputFile, '# calibration results: online constraint filtering\n');
        %         fprintf(OutputFile, '# format: q1 q2 q3 q4 x y\n');
        %
        %         if FlagDrawSet == 1
        %             err_std = zeros(6,1);
        %             err_mean = zeros(6,1);
        %             for i = 1:6
        %                 err_std(i) = std(recmat_x{i}(end,:));
        %                 err_mean(i) = mean(recmat_x{i}(end,:))-x_ref(i);
        %             end
        %
        %             fprintf(OutputFile, 'mean of error (s): ');
        %             for i = 1:6
        %                 fprintf(OutputFile, [num2str(err_mean(i)), ' ']);
        %             end
        %             fprintf(OutputFile, '\n');
        %             fprintf(OutputFile, 'std of error (s): ');
        %             for i = 1:6
        %                 fprintf(OutputFile, [num2str(err_std(i)), ' ']);
        %             end
        %             fprintf(OutputFile, '\n');
        %         end
        %
        %         if FlagDrawInit == 1
        %             err_std = zeros(6,1);
        %             err_mean = zeros(6,1);
        %             for i = 1:6
        %                 err_std(i) = std(recmat_x2{i}(end,:));
        %                 err_mean(i) = mean(recmat_x2{i}(end,:))-x_ref(i);
        %             end
        %
        %             fprintf(OutputFile, 'mean of error (i): ');
        %             for i = 1:6
        %                 fprintf(OutputFile, [num2str(err_mean(i)), ' ']);
        %             end
        %             fprintf(OutputFile, '\n');
        %             fprintf(OutputFile, 'std of error (i): ');
        %             for i = 1:6
        %                 fprintf(OutputFile, [num2str(err_std(i)), ' ']);
        %             end
        %             fprintf(OutputFile, '\n');
        %         end
        %
        %         fclose(OutputFile);
    end
end

