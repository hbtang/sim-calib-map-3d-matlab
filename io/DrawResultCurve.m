function DrawResultCurve( cell_results, vec_errodo, vec_errmk, ...
    options)
%DRAWRESULTCURVE Draw curves

if nargin < 4
    
    options.b_drawodo = true;
    options.b_drawimg = true;
    options.stdratio_odo = 0;
    options.std_img = 0;
    options.algorithms = {'initvo'; 'initmk'};
    options.property = 'norm_tvec';
    options.outputfilename = '.\temp\test';
end

[num_errodo, num_errmk] = size(cell_results);
num_algorithm = numel(options.algorithms);

switch options.property
    case 'norm_tvec'
        str_title_property = 'Trans. Err.';
        str_ylable = 'Trans. Err. (mm)';
    case 'norm_rvec'
        str_title_property = 'Rot. Err.';
        str_ylable = 'Rot. Err. (rad)';
    case 'dt_b_c'
        str_title_property = 'Temp. Delay';
        str_ylable = 'Temp. Delay (ms)';
    case 'k_odo_lin'
        str_title_property = 'Odo. Lin.';
        str_ylable = 'Odo. Lin.';
    case 'k_odo_rot'
        str_title_property = 'Odo. Rot.';
        str_ylable = 'Odo. Rot.';
end

%% draw curve with x = odoerrratio
if options.b_drawodo
    
    figure; grid on; hold on;
    str_xlabel = 'Odo. Std. Err. Ratio';
    str_title = [str_title_property, ' with Std. Img. Err. = ', num2str(options.std_img)];
    title(str_title, 'FontWeight','bold');
    xlabel(str_xlabel);
    ylabel(str_ylable);
    
    idx_errmk = find(vec_errmk == options.std_img, 1);
    if isempty(idx_errmk)
        error('Error in DrawResultCurve!');
    end
    
    mat_data = zeros(num_errodo, num_algorithm);
    for idx_algorithm = 1:num_algorithm
        fieldname_algorithm = ['error_', options.algorithms{idx_algorithm}];
        for idx_errodo = 1:num_errodo
            error_temp = cell_results{idx_errodo, idx_errmk}.(fieldname_algorithm);
            mat_data(idx_errodo, idx_algorithm) = error_temp.(options.property);
        end
    end
    
    plot(vec_errodo, mat_data, '-o');
    legend(options.algorithms, 'Location', 'best');
end

%% draw curve with x = errstdimg
if options.b_drawimg
    
    figure; grid on; hold on;
    str_xlabel = 'Image Err. Std. (pixel)';
    str_title = [str_title_property, ' with Odo. Std.Err.Ratio = ', num2str(options.stdratio_odo)];
    title(str_title, 'FontWeight','bold');
    xlabel(str_xlabel);
    ylabel(str_ylable);
    
    idx_errodo = find(vec_errodo == options.stdratio_odo, 1);
    if isempty(idx_errodo)
        error('Error in DrawResultCurve!');
    end
    
    mat_data = zeros(num_errodo, num_algorithm);
    for idx_algorithm = 1:num_algorithm
        fieldname_algorithm = ['error_', options.algorithms{idx_algorithm}];
        for idx_errmk = 1:num_errmk
            error_temp = cell_results{idx_errodo, idx_errmk}.(fieldname_algorithm);
            mat_data(idx_errmk, idx_algorithm) = error_temp.(options.property);
        end
    end
    
    plot(vec_errmk, mat_data, '-o');
    legend(options.algorithms, 'Location', 'best');
end

%% save figure
print([options.outputfilename, '.eps'], '-depsc', '-r0');
print([options.outputfilename, '.emf'], '-dmeta', '-r0');
print([options.outputfilename, '.jpg'], '-djpeg', '-r0');

end

