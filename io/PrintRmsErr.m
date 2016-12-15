function PrintRmsErr( struct_errors, cell_groups )
% to print rmse
disp('rmse:');
for idx_algor = 1:numel(cell_groups)
    str_algorithm = cell_groups{idx_algor};
    struct_error_temp = struct_errors.(str_algorithm);
    rms_img_temp = rms(struct_error_temp.mat_errPts);
    rms_odo_temp = rms(struct_error_temp.mat_errOdo);
    
    disp([ ...
        'algorithm: ', str_algorithm, ...
        '; img_u: ', num2str(rms_img_temp(1)), ...
        '; img_v: ', num2str(rms_img_temp(2)), ...
        '; odo_x: ', num2str(rms_odo_temp(1)), ...
        '; odo_y: ', num2str(rms_odo_temp(2)), ...
        '; odo_theta: ', num2str(rms_odo_temp(3))
        ]);
    
end
disp(' ');
end
