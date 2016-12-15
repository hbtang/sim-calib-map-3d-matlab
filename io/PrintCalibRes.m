function PrintCalibRes( struct_results, cell_groups )
% to print rmse
disp('calibration results:')
for idx_algor = 1:numel(cell_groups)
    str_algorithm = cell_groups{idx_algor};
    struct_calibres_temp = struct_results.(str_algorithm);
    
    disp(['algorithm: ', str_algorithm]);
    rvec_b_c = struct_calibres_temp.rvec_b_c;
    disp(['rvec_b_c: ', num2str(rvec_b_c(1)), ' ', ...
        num2str(rvec_b_c(2)), ' ', num2str(rvec_b_c(3)), ' ']);
    
    tvec_b_c = struct_calibres_temp.tvec_b_c;
    disp(['tvec_b_c: ', num2str(tvec_b_c(1)), ' ', ...
        num2str(tvec_b_c(2)), ' ', num2str(tvec_b_c(3)), ' ']);
    
    dt_b_c = struct_calibres_temp.dt_b_c;
    disp(['dt_b_c: ', num2str(dt_b_c), ' ']);
    
    k_odo_lin = struct_calibres_temp.k_odo_lin;
    k_odo_rot = struct_calibres_temp.k_odo_rot;
    disp(['k_odo_lin, k_odo_rot: ', ...
        num2str(k_odo_lin), ' ', num2str(k_odo_rot), ' ']);
    
    fx = struct_calibres_temp.mat_camera(1,1);
    fy = struct_calibres_temp.mat_camera(2,2);
    cx = struct_calibres_temp.mat_camera(1,3);
    cy = struct_calibres_temp.mat_camera(2,3);
    disp(['fx fy cx cy: ', ...
        num2str(fx), ' ', num2str(fy), ' ', ...
        num2str(cx), ' ', num2str(cy), ' ']);
    
    distortion = struct_calibres_temp.vec_distortion;
    disp(['distortion: ', num2str(distortion(1)), ' ', ...
        num2str(distortion(2)), ' ', num2str(distortion(3)), ' ', ...
        num2str(distortion(4)), ' ', num2str(distortion(5)), ' ']);
    
    disp(' ');
end
disp(' ');
end
