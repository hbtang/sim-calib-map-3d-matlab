%% init
clear;

% referece
x_ref = [0;0;1/sqrt(2);-1/sqrt(2);0;0];

% path
PathFold = '..\data\sim-sqrmap-inout-20160118\';
PathFoldRes = [PathFold, 'res\'];

% number of record
InitNum = 50;
SetNum = 50;

% Ns configure
MkNsSt = 0; MkNsEnd = 5; MkNsNum = MkNsEnd-MkNsSt+1;
OdoNsSt = 0; OdoNsEnd = 5; OdoNsNum = OdoNsEnd-OdoNsSt+1;

% figure configure
FigPos = [1 1 480 360];

%% read data

if exist('.\temp\rec_sim_final.mat', 'file') == 0
    
    cell_rec_x_s_final = cell(MkNsNum,OdoNsNum);
    cell_rec_x_i_final = cell(MkNsNum,OdoNsNum);
    
    for MkNs = MkNsSt:MkNsEnd
        for OdoNs = OdoNsSt:OdoNsEnd
            
            disp([num2str(MkNs), num2str(OdoNs)]);
            
            NameMkStr = ['Mk-z', num2str(MkNs), '-xy', num2str(MkNs)];
            NameOdoStr = ['Odo-l', num2str(OdoNs), '-r', num2str(OdoNs)];
            
            %% read set data
            rec_x_s_final = zeros(SetNum, 6);
            for i = 1:SetNum
                SetId = i;
                PathRecData = [PathFoldRes, 'res-', NameMkStr, '-', NameOdoStr,...
                    '-s', num2str(SetId), '-i', num2str(1), '.mat'];
                load(PathRecData);
                rec_x_s_final(i, :) = rec_x(end, :);
            end
            cell_rec_x_s_final{MkNs+1, OdoNs+1} = rec_x_s_final;
            
            %% read init data
            rec_x_i_final = zeros(SetNum, 6);
            for i = 1:InitNum
                InitId = i;
                PathRecData = [PathFoldRes, 'res-',NameMkStr, '-', NameOdoStr,...
                    '-s', num2str(1), '-i', num2str(InitId), '.mat'];
                load(PathRecData);
                rec_x_i_final(i, :) = rec_x(end, :);
            end
            cell_rec_x_i_final{MkNs+1, OdoNs+1} = rec_x_i_final;
            
        end
    end
    save('.\temp\rec_sim_final.mat', 'cell_rec_x_s_final', 'cell_rec_x_i_final');
else
    load('.\temp\rec_sim_final.mat');
end

%% compute a-rms, r-rms, and num-outlier for set data

sz = size(cell_rec_x_s_final);
threshQuat = 0.3;
threshXY = 300;

% for init

mat_numOutlier_i = zeros(sz(1), sz(2));
mat_aRMS_rot_i = zeros(sz(1), sz(2));
mat_aRMS_trans_i = zeros(sz(1), sz(2));
mat_rRMS_rot_i = zeros(sz(1), sz(2));
mat_rRMS_trans_i = zeros(sz(1), sz(2));

for i = 1:sz(1)
    for j = 1:sz(2)
        rec_x_final_temp = cell_rec_x_i_final{i,j};
        
        %% outliers
        vec_idx_outlier = [];
        for k = 1:numel(rec_x_final_temp(:,1))
            err_temp = rec_x_final_temp(k,:).' - x_ref;
            if norm(err_temp(1:4)) > threshQuat || norm(err_temp(5:6)) > threshXY
                vec_idx_outlier = [vec_idx_outlier; k];
            end
        end
        % record outlier matrix
        mat_numOutlier_i(i,j) = numel(vec_idx_outlier);
        % prune outliers
        rec_x_final_temp(vec_idx_outlier, :) = [];
        
        %% aRMS
        err_a = rec_x_final_temp - repmat(x_ref.', numel(rec_x_final_temp(:,1)),1);
        mat_aRMS_rot_i(i,j) = rms(rms(err_a(:, 1:4)));
        mat_aRMS_trans_i(i,j) = rms(rms(err_a(:, 5:6)));
        
        %% rRMS
        x_avr = mean(rec_x_final_temp).';
        err_r = rec_x_final_temp - repmat(x_avr.', numel(rec_x_final_temp(:,1)),1);
        mat_rRMS_rot_i(i,j) = rms(rms(err_r(:, 1:4)));
        mat_rRMS_trans_i(i,j) = rms(rms(err_r(:, 5:6)));
        
    end
end

% for sets

mat_numOutlier_s = zeros(sz(1), sz(2));
mat_aRMS_rot_s = zeros(sz(1), sz(2));
mat_aRMS_trans_s = zeros(sz(1), sz(2));
mat_rRMS_rot_s = zeros(sz(1), sz(2));
mat_rRMS_trans_s = zeros(sz(1), sz(2));

for i = 1:sz(1)
    for j = 1:sz(2)
        rec_x_final_temp = cell_rec_x_s_final{i,j};
        
        %% outliers
        vec_idx_outlier = [];
        for k = 1:numel(rec_x_final_temp(:,1))
            err_temp = rec_x_final_temp(k,:).' - x_ref;
            if norm(err_temp(1:4)) > threshQuat || norm(err_temp(5:6)) > threshXY
                vec_idx_outlier = [vec_idx_outlier; k];
            end
        end
        % record outlier matrix
        mat_numOutlier_s(i,j) = numel(vec_idx_outlier);
        % prune outliers
        rec_x_final_temp(vec_idx_outlier, :) = [];
        
        %% aRMS
        err_a = rec_x_final_temp - repmat(x_ref.', numel(rec_x_final_temp(:,1)),1);
        mat_aRMS_rot_s(i,j) = rms(rms(err_a(:, 1:4)));
        mat_aRMS_trans_s(i,j) = rms(rms(err_a(:, 5:6)));
        
        %% rRMS
        x_avr = mean(rec_x_final_temp).';
        err_r = rec_x_final_temp - repmat(x_avr.', numel(rec_x_final_temp(:,1)),1);
        mat_rRMS_rot_s(i,j) = rms(rms(err_r(:, 1:4)));
        mat_rRMS_trans_s(i,j) = rms(rms(err_r(:, 5:6)));
        
    end
end

%% show bar3 set
% outliers
mat_data = mat_numOutlier_s;
fileNameFigOutput = '.\temp\numOutlier-final-s';
strZLabel = 'Num. Outliers';
strTitle = 'Final Result: Num. Outliers';
Bar3SimFinal( mat_data, fileNameFigOutput, strZLabel, strTitle, FigPos);

% aRMSE-rot
mat_data = mat_aRMS_rot_s;
fileNameFigOutput = '.\temp\aRmse-rot-final-s';
strZLabel = 'Rot. aRMSE (rad)';
strTitle = 'Final Result: Rot. aRMSE';
Bar3SimFinal( mat_data, fileNameFigOutput, strZLabel, strTitle, FigPos);

% aRMSE-trans
mat_data = mat_aRMS_trans_s;
fileNameFigOutput = '.\temp\aRmse-trans-final-s';
strZLabel = 'Trans. aRMSE (mm)';
strTitle = 'Final Result: Trans. aRMSE';
Bar3SimFinal( mat_data, fileNameFigOutput, strZLabel, strTitle, FigPos);

% rRMSE-rot
mat_data = mat_rRMS_rot_s;
fileNameFigOutput = '.\temp\rRmse-rot-final-s';
strZLabel = 'Rot. rRMSE (rad)';
strTitle = 'Final Result: Rot. rRMSE';
Bar3SimFinal( mat_data, fileNameFigOutput, strZLabel, strTitle, FigPos);

% rRMSE-trans
mat_data = mat_rRMS_trans_s;
fileNameFigOutput = '.\temp\rRmse-trans-final-s';
strZLabel = 'Trans. rRMSE (mm)';
strTitle = 'Final Result: Trans. rRMSE';
Bar3SimFinal( mat_data, fileNameFigOutput, strZLabel, strTitle, FigPos);

%% show bar3 init
% outliers
mat_data = mat_numOutlier_i;
fileNameFigOutput = '.\temp\numOutlier-final-i';
strZLabel = 'Num. Outliers';
strTitle = 'Final Result: Num. Outliers';
Bar3SimFinal( mat_data, fileNameFigOutput, strZLabel, strTitle, FigPos);

% aRMSE-rot
mat_data = mat_aRMS_rot_i;
fileNameFigOutput = '.\temp\aRmse-rot-final-i';
strZLabel = 'Rot. aRMSE (rad)';
strTitle = 'Final Result: Rot. aRMSE';
Bar3SimFinal( mat_data, fileNameFigOutput, strZLabel, strTitle, FigPos);

% aRMSE-trans
mat_data = mat_aRMS_trans_i;
fileNameFigOutput = '.\temp\aRmse-trans-final-i';
strZLabel = 'Trans. aRMSE (mm)';
strTitle = 'Final Result: Trans. aRMSE';
Bar3SimFinal( mat_data, fileNameFigOutput, strZLabel, strTitle, FigPos);

% rRMSE-rot
mat_data = mat_rRMS_rot_i;
fileNameFigOutput = '.\temp\rRmse-rot-final-i';
strZLabel = 'Rot. rRMSE (rad)';
strTitle = 'Final Result: Rot. rRMSE';
Bar3SimFinal( mat_data, fileNameFigOutput, strZLabel, strTitle, FigPos);

% rRMSE-trans
mat_data = mat_rRMS_trans_i;
fileNameFigOutput = '.\temp\rRmse-trans-final-i';
strZLabel = 'Trans. rRMSE (mm)';
strTitle = 'Final Result: Trans. rRMSE';
Bar3SimFinal( mat_data, fileNameFigOutput, strZLabel, strTitle, FigPos);





