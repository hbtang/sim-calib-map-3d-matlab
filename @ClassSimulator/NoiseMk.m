function mk_out = NoiseMk(this, mk_true, calib, setting)

num_mk = numel(mk_true.lp);
mk_out = mk_true;
error = setting.error;

%% add noise into image
std_imgu = error.mk.std_imgu;
std_imgv = error.mk.std_imgv;
stdimage = [std_imgu;std_imgv;std_imgu;std_imgv;std_imgu;std_imgv;std_imgu;std_imgv];
for i = 1:num_mk
    imageTemp = mk_out.image(i,:).';
    image_noise_temp = normrnd(imageTemp, stdimage);
    mk_out.image(i,:) = image_noise_temp.';
end

%% compute rvec and tvec with noised image

mat_camera = calib.mat_camera;
vec_distortion = calib.vec_distortion;
IntrinsicMatrix = mat_camera.';
RadialDistortion = [vec_distortion(1) vec_distortion(2) vec_distortion(5)];
TangentialDistortion = [vec_distortion(3) vec_distortion(4)];
cameraParams = cameraParameters(...
    'IntrinsicMatrix', IntrinsicMatrix, ...
    'RadialDistortion', RadialDistortion, ...
    'TangentialDistortion', TangentialDistortion);

worldPoints = setting.aruco.mattvec;

rows_good = [];
thresh_err = 1;
% rec_error = [];

for i = 1:num_mk
    
    if i == 246
        debug = 1;
    end
    
    %% solve pnp by solveP3P algorithm
    imageTemp = mk_out.image(i,:);
    pointsTemp = [...
        imageTemp(1:2); ...
        imageTemp(3:4); ...
        imageTemp(5:6); ...
        imageTemp(7:8)];
    pointsUndistorted = undistortPoints(pointsTemp, cameraParams);
    
    [Rs_m_c, Ts_c_m] = vision.internal.calibration.solveP3P(...
        pointsUndistorted, worldPoints, cameraParams.IntrinsicMatrix);
    
    %% choose the best solution
    [~,~,numSolutions] = size(Rs_m_c);
    rvec_c_m_true = mk_out.rvec(i,:).';
    R_c_m_true = rodrigues(rvec_c_m_true);
    tvec_c_m_true = mk_out.tvec(i,:).';
    
    idx_bestsolution = 0;
    error_bestsolution = inf;
    rvec_c_m_best = [];
    tvec_c_m_best = [];
    for j = 1:numSolutions
        R_m_c_j = Rs_m_c(:,:,j);
        R_c_m_j = R_m_c_j.';
        rvec_c_m_j = rodrigues(R_c_m_j);
        tvec_c_m_j = Ts_c_m(j,:).';
        
        dR = R_c_m_j.'*R_c_m_true;
        drvec = rodrigues(dR);
        dtvec = tvec_c_m_true - tvec_c_m_j;
        error = norm(drvec);
        
        if error < error_bestsolution
            error_bestsolution = error;
            idx_bestsolution = j;
            rvec_c_m_best = rvec_c_m_j;
            tvec_c_m_best = tvec_c_m_j;
        end
    end    
    
    %% record
    if idx_bestsolution == 0
        continue;
    end
    mk_out.rvec(i,:) = rvec_c_m_best.';
    mk_out.tvec(i,:) = tvec_c_m_best.';    
    if error_bestsolution < thresh_err
        rows_good = [rows_good; i];
    end
    
    %% debug
%     rec_error = [rec_error; error_bestsolution];
end

%% prune bad mark
mk_out.lp = mk_out.lp(rows_good);
mk_out.id = mk_out.id(rows_good);
mk_out.rvec = mk_out.rvec(rows_good,:);
mk_out.tvec = mk_out.tvec(rows_good,:);
mk_out.image = mk_out.image(rows_good,:);

end

