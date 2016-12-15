function ScatterErr( struct_errors, cell_groups, options )
%SCATTERERRIMG 此处显示有关此函数的摘要
% options.flag: 1, image error; 2, odox-odoy; 3, odox-odotheta
% options.b_saveimg: true, save image;
% options.outputfilename: string of image file name

figure;

flag = options.flag;
switch flag
    case 1
        axis equal;
        str_title = 'Reprojected Error: Vision';
        str_xlabel = 'Img. Err. X (pixel)';
        str_ylabel = 'Img. Err. Y (pixel)';
    case 2
        axis equal;
        str_title = 'Reprojected Error: Odometry';
        str_xlabel = 'Odo. Err. X (mm)';
        str_ylabel = 'Odo. Err. Y (mm)';
    case 3
        str_title = 'Reprojected Error: Odometry';
        str_xlabel = 'Odo. Err. X (mm)';
        str_ylabel = 'Odo. Err. Theta (rad)';
end

% title(str_title, 'FontWeight','bold');
% xlabel(str_xlabel);
% ylabel(str_ylabel);

mat_data = [];
cell_group = {};
for idx_algor = 1:numel(cell_groups)
    str_algorithm = cell_groups{idx_algor};
    struct_error_temp = struct_errors.(str_algorithm);
    
    switch flag
        case 1
            mat_data_temp = struct_error_temp.mat_errPts;
            mat_data_temp = RemoveOutlier(mat_data_temp, 3, [200 200; -200 -200]);
        case 2
            mat_data_temp = struct_error_temp.mat_errOdo(:,[1,2]);
            mat_data_temp = RemoveOutlier(mat_data_temp, 3, [500 500; -500 -500]);
        case 3
            mat_data_temp = struct_error_temp.mat_errOdo(:,[1,3]);
            mat_data_temp = RemoveOutlier(mat_data_temp, 3, [500 0.1; -500 -0.1]);
    end   
    
    mat_data = [mat_data; mat_data_temp];
    [numrow_data_temp, ~] = size(mat_data_temp);
    
    cell_group_temp = cell(numrow_data_temp, 1);
    for j = 1:numrow_data_temp
        cell_group_temp{j,1} = str_algorithm;
    end
    cell_group = [cell_group; cell_group_temp];
end

h_sh = scatterhist(mat_data(:,1), mat_data(:,2), 'Group', cell_group, 'Marker', '.');
h_sh(1).XLabel.String = str_xlabel;
h_sh(1).YLabel.String = str_ylabel;
h_sh(1).Title.String = str_title;
h_sh(1).Title.FontWeight = 'bold';
grid on;


%% save
if options.b_saveimg
    print([options.outputfilename, '.eps'], '-depsc', '-r0');
    print([options.outputfilename, '.emf'], '-dmeta', '-r0');
    print([options.outputfilename, '.jpg'], '-djpeg', '-r0');
end

end

