function SolveJointOptVSlam(this, measure, calib, map, setting, options)

%% init

if nargin < 6
    options = [];
end

if ~isfield(options, 'bCalibExtRot')
    options.bCalibExtRot = true;
end
if ~isfield(options, 'bCalibExtLin')
    options.bCalibExtLin = true;
end
if ~isfield(options, 'bCalibTmp')
    options.bCalibTmp = true;
end
if ~isfield(options, 'bCalibOdo')
    options.bCalibOdo = true;
end
if ~isfield(options, 'bCalibCamMat')
    options.bCalibCamMat = false;
end
if ~isfield(options, 'bCalibCamDist')
    options.bCalibCamDist = false;
end

disp(['Start calibration with V-SLAM...']);

mk = measure.mk;
odo = measure.odo;
time = measure.time;

% init state vector q ...
idxStMk = options.bCalibExtRot*3 + options.bCalibExtLin*2 ...
    + options.bCalibTmp + options.bCalibOdo*2 ...
    + options.bCalibCamMat*4 + options.bCalibCamDist*5;
q = zeros(idxStMk+6*mk.numMkId+3*odo.num,1);
q_init = [];
count = 1;
if options.bCalibExtRot
    q_init_extrot = calib.rvec_b_c;
    q_init = [q_init; q_init_extrot];
    vecrow_extrot = count:count+2;
    count = count + 3;
end
if options.bCalibExtLin
    q_init_extlin = calib.tvec_b_c(1:2);
    q_init = [q_init; q_init_extlin];
    vecrow_extlin = count:count+1;
    count = count + 2;
end
if options.bCalibTmp
    q_init_tmp = calib.dt;
    q_init = [q_init; q_init_tmp];
    vecrow_tmp = count;
    count = count + 1;
end
if options.bCalibOdo
    q_init_odo = [calib.k_odo_lin; calib.k_odo_rot];
    q_init = [q_init; q_init_odo];
    vecrow_odo = count:count+1;
    count = count + 2;
end
if options.bCalibCamMat
    fx = calib.mat_camera(1,1);
    fy = calib.mat_camera(2,2);
    cx = calib.mat_camera(1,3);
    cy = calib.mat_camera(2,3);
    q_init_cammat = [fx; fy; cx; cy];
    q_init = [q_init; q_init_cammat];
    vecrow_cammat = count:count+3;
    count = count + 4;
end
if options.bCalibCamDist
    k1 = calib.vec_distortion(1);
    k2 = calib.vec_distortion(2);
    p1 = calib.vec_distortion(3);
    p2 = calib.vec_distortion(4);
    k3 = calib.vec_distortion(5); 
    q_init_camdist = [k1; k2; p1; p2; k3];
    q_init = [q_init; q_init_camdist];
    vecrow_camdist = count:count+4;
    count = count + 5;
end
q(1:idxStMk) = q_init;
for i = 1:mk.numMkId
    q(idxStMk+i*6-5:idxStMk+i*6-3) = map.mks.rvec_w_m(i,:).';
    q(idxStMk+i*6-2:idxStMk+i*6) = map.mks.tvec_w_m(i,:).';
end
idxStOdo = idxStMk+6*mk.numMkId;
for i = 1:odo.num
    q(idxStOdo+3*i-2: idxStOdo+3*i) = map.kfs.ps2d_w_b(i,:).';
end

%% solve nonlinear least square, mapping and calibration
options_optim = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
    'Display', 'iter-detailed', 'Jacobian', 'on', 'MaxIter', 50, 'ScaleProblem', 'Jacobian', 'TolX', 1e-6);
[q,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(...
    @(x)this.CostJointOptVSlam(x, mk, odo, time, calib, setting, options), q, [], [], options_optim);

% options.Algorithm = 'trust-region-reflective';
% [q,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(...
%     @(x)this.CostJointOpt5(x, mk, odo, time, calib, setting, flag), q, [], [], options);

%% save results

% refresh calib
if options.bCalibExtRot
    q_extrot = q(vecrow_extrot);
    calib.rvec_b_c = q_extrot;
    calib.RefreshByVecbc;
end
if options.bCalibExtLin
    q_extlin = q(vecrow_extlin);
    calib.tvec_b_c = [q_extlin;0];
    calib.RefreshByVecbc;
end
if options.bCalibTmp
    calib.dt = q(vecrow_tmp);
end
if options.bCalibOdo
    q_odo = q(vecrow_odo);
    calib.k_odo_lin = q_odo(1);
    calib.k_odo_rot = q_odo(2);
end
if options.bCalibCamMat
    q_cammat = q(vecrow_cammat);    
    fx = q_cammat(1);
    fy = q_cammat(2);
    cx = q_cammat(3);
    cy = q_cammat(4);  
    calib.mat_camera = [fx 0 cx; 0 fy cy; 0 0 1];
end
if options.bCalibCamDist
    q_camdist = q(vecrow_camdist);    
    k1 = q_camdist(1);
    k2 = q_camdist(2);
    p1 = q_camdist(3);
    p2 = q_camdist(4);
    k3 = q_camdist(5);    
    calib.vec_distortion = [k1; k2; p1; p2; k3];
end

% refresh map
mat_tvec_w_m = zeros(mk.numMkId, 3);
mat_rvec_w_m = zeros(mk.numMkId, 3);
for i = 1:mk.numMkId
    mat_rvec_w_m(i,:) = q(idxStMk+6*i-5:idxStMk+6*i-3).';
    mat_tvec_w_m(i,:) = q(idxStMk+6*i-2:idxStMk+6*i).';
end

vecPs2d_w_b = zeros(odo.num, 3);
for i = 1:odo.num
    vecPs2d_w_b(i,:) = q(idxStOdo+3*i-2:idxStOdo+3*i).';
end

map.mks.rvec_w_m = mat_rvec_w_m;
map.mks.tvec_w_m = mat_tvec_w_m;
map.kfs.ps2d_w_b = vecPs2d_w_b;
map.RefreshKfsByPs2dwb;

disp('End calibration with V-SLAM.');
disp(' ');

end

