function SolveJointOptMSlam(this, measure, calib, map, setting, options)

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

disp(['Start calibration with M-SLAM...']);

mk = measure.mk;
odo = measure.odo;
time = measure.time;

% init state vector q ...
idxStMk = options.bCalibExtRot*3 + options.bCalibExtLin*2 + options.bCalibTmp + options.bCalibOdo*2;
q = zeros(idxStMk+3*mk.numMkId+3*odo.num,1);
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
q(1:idxStMk) = q_init;
for i = 1:mk.numMkId
    q(idxStMk+i*3-2:idxStMk+i*3) = map.mks.tvec_w_m(i,:).';
end
idxStOdo = idxStMk+3*mk.numMkId;
for i = 1:odo.num
    q(idxStOdo+3*i-2: idxStOdo+3*i) = map.kfs.ps2d_w_b(i,:).';
end

%% solve nonlinear least square, mapping and calibration
options_optim = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
    'Display', 'iter-detailed', 'Jacobian', 'on', 'MaxIter', 50, 'ScaleProblem', 'Jacobian', 'TolX', 1e-6);
[q,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(...
    @(x)this.CostJointOptMSlam(x, mk, odo, time, calib, setting, options), q, [], [], options_optim);

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

% refresh map
mat_tvec_w_m = zeros(mk.numMkId, 3);
for i = 1:mk.numMkId
    mat_tvec_w_m(i,:) = q(idxStMk+3*i-2:idxStMk+3*i).';
end

vecPs2d_w_b = zeros(odo.num, 3);
for i = 1:odo.num
    vecPs2d_w_b(i,:) = q(idxStOdo+3*i-2:idxStOdo+3*i).';
end

map.mks.tvec_w_m = mat_tvec_w_m;
map.kfs.ps2d_w_b = vecPs2d_w_b;
map.RefreshKfsByPs2dwb;

disp('End calibration with M-SLAM.');
disp(' ');

end

