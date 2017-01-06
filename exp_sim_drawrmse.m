%% init
close all;

% truth
truth.rvec_b_c = [0, 2.2214, -2.2214].';
truth.tvec_b_c = [0.0, 0.0, 0.0].';
truth.dt_b_c = 0.01;
truth.k_odo_lin = 1.01;
truth.k_odo_rot = 1.01;

% data path
str_datapath = '.\sim\results_spodotemp\';

% load algorithms
bCalibSp = true;
bCalibSpOdo = false;
bCalibSpOdoTmp = true;

% draw configure
b_drawimg = false;
stdratio_odo = 0.0;
b_drawodo = true;
std_img = 0;
if bCalibSp && bCalibSpOdo && bCalibSpOdoTmp
    vec_flag_algor = [1,2,3,4];
elseif bCalibSp && bCalibSpOdoTmp
    vec_flag_algor = [1,2];
elseif bCalibSp
    vec_flag_algor = [1];
end

% select data
if b_drawimg
    stdratio_odo_st = stdratio_odo;
    stdratio_odo_end = stdratio_odo;
    stdratio_odo_step = 0.001;
    std_img_st = 0;
    std_img_end = 1;
    std_img_step = 0.025;
elseif b_drawodo
    stdratio_odo_st = 0;
    stdratio_odo_end = 0.04;
    stdratio_odo_step = 0.001;
    std_img_st = std_img;
    std_img_end = std_img;
    std_img_step = 0.025;
end

% select set
idx_set_st = 1;
idx_set_end = 20;

%% load

cell_results = cell(0,0,0);

vec_errodo = (stdratio_odo_st:stdratio_odo_step:stdratio_odo_end).';
vec_errmk = (std_img_st:std_img_step:std_img_end).';

for idx_set = idx_set_st:idx_set_end
    idx_row = 0;
    for errratio_odo = vec_errodo.'
        idx_row = idx_row + 1;
        
        set_odo = idx_set;
        stdratio_lin = errratio_odo;
        stdratio_rot = errratio_odo;
        str_suffix_odo = [ ...
            '-el', num2str(stdratio_lin), ...
            '-er', num2str(stdratio_rot), ...
            '-s', num2str(set_odo), ...
            ];
        
        idx_col = 0;
        for std_img = vec_errmk.'
            idx_col = idx_col + 1;
            
            set_img = idx_set;
            std_imgu = std_img;
            std_imgv = std_img;
            str_suffix_mk = [ ...
                '-eu', num2str(std_imgu), ...
                '-ev', num2str(std_imgv), ...
                '-s', num2str(set_img), ...
                ];
            str_file_record = [str_datapath, 'results', str_suffix_odo, str_suffix_mk, '.mat'];
            load(str_file_record);
            
            cell_results{idx_row, idx_col, idx_set}.stdratio_lin = stdratio_lin;
            cell_results{idx_row, idx_col, idx_set}.stdratio_rot = stdratio_rot;
            cell_results{idx_row, idx_col, idx_set}.std_imgu = std_imgu;
            cell_results{idx_row, idx_col, idx_set}.std_imgv = std_imgv;
            
            cell_results{idx_row, idx_col, idx_set}.result_initvo = struct_record.result_initvo;
            cell_results{idx_row, idx_col, idx_set}.result_initmk = struct_record.result_initmk;
            cell_results{idx_row, idx_col, idx_set}.error_initvo = GetCalibError( struct_record.result_initvo, truth );
            cell_results{idx_row, idx_col, idx_set}.error_initmk = GetCalibError( struct_record.result_initmk, truth );
            
            if bCalibSp
                cell_results{idx_row, idx_col, idx_set}.result_mslam = struct_record.result_mslam;
                cell_results{idx_row, idx_col, idx_set}.result_vslam = struct_record.result_vslam;
                cell_results{idx_row, idx_col, idx_set}.error_mslam = GetCalibError( struct_record.result_mslam, truth );
                cell_results{idx_row, idx_col, idx_set}.error_vslam = GetCalibError( struct_record.result_vslam, truth );
            end
            
            if bCalibSpOdo
                cell_results{idx_row, idx_col, idx_set}.result_mslamodo = struct_record.result_mslamodo;
                cell_results{idx_row, idx_col, idx_set}.result_vslamodo = struct_record.result_vslamodo;
                cell_results{idx_row, idx_col, idx_set}.error_mslamodo = GetCalibError( struct_record.result_mslamodo, truth );
                cell_results{idx_row, idx_col, idx_set}.error_vslamodo = GetCalibError( struct_record.result_vslamodo, truth );
            end
            
            if bCalibSpOdoTmp
                cell_results{idx_row, idx_col, idx_set}.result_mslamodotemp = struct_record.result_mslamodotemp;
                cell_results{idx_row, idx_col, idx_set}.result_vslamodotemp = struct_record.result_vslamodotemp;
                cell_results{idx_row, idx_col, idx_set}.error_mslamodotemp = GetCalibError( struct_record.result_mslamodotemp, truth );
                cell_results{idx_row, idx_col, idx_set}.error_vslamodotemp = GetCalibError( struct_record.result_vslamodotemp, truth );
            end
        end
    end
    
end

%% draw

for flag_algorithm = vec_flag_algor
    options_draw = struct( ...
        'b_drawodo', b_drawodo, ...
        'b_drawimg', b_drawimg, ...
        'stdratio_odo', stdratio_odo, ...
        'std_img', std_img);
    
    switch flag_algorithm
        case 1
            options_draw.algorithms = { ...
                'initvo', 'initmk', ...
                'mslam', 'vslam', ...
                'mslamodotemp', 'vslamodotemp'};
        case 2
            options_draw.algorithms = { ...                
                'mslamodotemp', 'vslamodotemp'};
        case 3            
        case 4            
    end
    
    if b_drawimg
        outputfilename_prefix = ['.\temp\sim-x_img-stdrodo_', num2str(stdratio_odo), ...
            '-algor_', num2str(flag_algorithm)];
    elseif b_drawodo
        outputfilename_prefix = ['.\temp\sim-x_odo-stdimg_', num2str(std_img), ...
            '-algor_', num2str(flag_algorithm)];
    end
    
    options_draw.property = 'norm_tvec';
    options_draw.outputfilename = [outputfilename_prefix, '-y_tvec'];
    DrawResultRmse( cell_results, vec_errodo, vec_errmk, options_draw);
    
    options_draw.property = 'norm_rvec';
    options_draw.outputfilename = [outputfilename_prefix, '-y_rvec'];
    DrawResultRmse( cell_results, vec_errodo, vec_errmk, options_draw);
    
    options_draw.property = 'dt_b_c';
    options_draw.outputfilename = [outputfilename_prefix, '-y_dt'];
    DrawResultRmse( cell_results, vec_errodo, vec_errmk, options_draw);
    
    options_draw.property = 'k_odo_lin';
    options_draw.outputfilename = [outputfilename_prefix, '-y_klin'];
    DrawResultRmse( cell_results, vec_errodo, vec_errmk, options_draw);
    
    options_draw.property = 'k_odo_rot';
    options_draw.outputfilename = [outputfilename_prefix, '-krot'];
    DrawResultRmse( cell_results, vec_errodo, vec_errmk, options_draw);
    
end





