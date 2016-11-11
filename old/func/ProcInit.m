function [mark, odo] = ProcInit(datasetId)

%% load landmark txt files
% dataSerialNum = '20160102';
landmarkFileName = ['data/rec_' num2str(datasetId) '/rec_mk.txt'];
markFileId = fopen(landmarkFileName,'r');
mark.stamp = [];
mark.id = [];
mark.rvec = [];
mark.tvec = [];
mark.num = 0;
mark.size = 120;
while ~feof(markFileId)
    line_temp = fgetl(markFileId);
    Cell_temp = strsplit(line_temp,{':',';','[',']'});
    if size(Cell_temp) == [1 13]
        line_vec = str2double(Cell_temp);
        stamp = line_vec(2);
        if stamp == 0
            continue;
        end
        id = line_vec(4);
%         if id > 75
%             continue;
%         end        
        rvec = line_vec(6:8);
        tvec = line_vec(10:12);
        mark.stamp = [mark.stamp; stamp];
        mark.id = [mark.id; id];
        mark.rvec = [mark.rvec; rvec];
        mark.tvec = [mark.tvec; tvec];        
    else
        error('Input file error: format error in mark record file!')
    end
end
fclose(markFileId);
mark.num = numel(mark.stamp);
% mark.stampMk = (1:mark.num).';
disp('marker info loaded.');

%% load odometry record file
RowCountReadingFile = 0;
odoFileName = ['data/rec_' num2str(datasetId) '/rec_odo.txt'];
odoFileId = fopen(odoFileName,'r');
odo.stamp = [];
odo.x = [];
odo.y = [];
odo.theta = [];
odo.num = 0;
while ~feof(odoFileId)
    line_temp = fgetl(odoFileId);
    Cell_temp = strsplit(line_temp,{':',';','='});
    if size(Cell_temp) == [1 15]
        stamp = str2double(Cell_temp(2));
        if stamp == 0
            continue;
        end        
        x = str2double(Cell_temp(4));
        y = str2double(Cell_temp(6));
        theta = str2double(Cell_temp(8));
        odo.stamp = [odo.stamp; stamp];
        odo.x = [odo.x; x];
        odo.y = [odo.y; y];
        odo.theta = [odo.theta; theta];
        RowCountReadingFile = RowCountReadingFile + 1;
        if(mod(RowCountReadingFile,100) == 0)
            disp(['loop ' num2str(RowCountReadingFile) ' odometry info loaded.']);
        end
    else
        error('Input file error: format error in odometry record file!')
    end
end
fclose(odoFileId);
odo.num = numel(odo.stamp);


