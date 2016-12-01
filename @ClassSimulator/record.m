function Record( this, options )
%RECORD write into record file Odo.rec and Mk.rec
if nargin < 2
    options = [];
end
if ~isfield(options, 'str_suffix_odo')
    options.str_suffix_odo = [];
end
if ~isfield(options, 'str_suffix_mk')
    options.str_suffix_mk = [];
end
if ~isfield(options, 'b_record_odo')
    options.b_record_odo = false;
end
if ~isfield(options, 'b_record_mk')
    options.b_record_mk = false;
end

% second
timestep = this.timestep;

%% print Odo.rec
if options.b_record_odo
    % open file
    this.files.file_out_odo = fopen([this.files.str_simfolder, 'rec/Odo', options.str_suffix_odo, '.rec'],'w+');
    fprintf(this.files.file_out_odo, '# odometry info\n');
    fprintf(this.files.file_out_odo, '# format: lp timeOdo timeCam x y theta\n');
    
    % record data
    odo_rec = this.odo_noise;
    for i = 1:numel(odo_rec.lp)
        strOdo = [num2str(odo_rec.lp(i)), ' '];
        strOdo = [strOdo, ...
            num2str(odo_rec.lp(i)*timestep), ' ', ...
            num2str(odo_rec.lp(i)*timestep), ' '];
        strOdo = [strOdo, ...
            num2str(odo_rec.x(i)), ' ', ...
            num2str(odo_rec.y(i)), ' ', ...
            num2str(odo_rec.theta(i)), '\n',];
        fprintf(this.files.file_out_odo, strOdo);
    end
    
    % close file
    fclose(this.files.file_out_odo);
end

%% print Mk.rec
if options.b_record_mk
    % open file
    this.files.file_out_mk = fopen([this.files.str_simfolder, 'rec/Mk', options.str_suffix_mk, '.rec'],'w+');
    fprintf(this.files.file_out_mk, '# aruco mark observation info\n');
    fprintf(this.files.file_out_mk, '# format: lp id rvec(x y z) tvec(x y z) ptimg(x1 y1 x2 y2 x3 y3 x4 y4)\n');
    
    % record data
    mk_rec = this.mk_noise;
    for i = 1:numel(mk_rec.lp)
        lp = mk_rec.lp(i);
        id = mk_rec.id(i);
        rvec = mk_rec.rvec(i,:).';
        tvec = mk_rec.tvec(i,:).';
        image = mk_rec.image(i,:).';
        strMk = [num2str(lp), ' ', num2str(id), ' '];
        strMk = [strMk, num2str(rvec(1)), ' ', num2str(rvec(2)), ' ', ...
            num2str(rvec(3)), ' ',];
        strMk = [strMk, num2str(tvec(1)), ' ', num2str(tvec(2)), ' ', ...
            num2str(tvec(3)), ' ',];
        strMk = [strMk, ...
            num2str(image(1)), ' ', ...
            num2str(image(2)), ' ', ...
            num2str(image(3)), ' ', ...
            num2str(image(4)), ' ', ...
            num2str(image(5)), ' ', ...
            num2str(image(6)), ' ', ...
            num2str(image(7)), ' ', ...
            num2str(image(8)), ' '];
        strMk = [strMk '\n'];
        fprintf(this.files.file_out_mk, strMk);
    end
    
    % close file
    fclose(this.files.file_out_mk);
end

end

