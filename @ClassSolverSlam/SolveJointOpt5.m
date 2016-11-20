function SolveJointOpt5(this, measure, calib, map, setting, flag)

%% init

if nargin < 6
    flag.bCalibExt = 1;
    flag.bCalibTmp = 1;
    flag.bCalibOdo = 1;
end

disp(['Start calibration with V-SLAM...']);

mk = measure.mk;
odo = measure.odo;
time = measure.time;

% define variable vector vec_q:
% [rvec_b_c; x_b_c; y_b_c, dt_b_c; [map.rvec_w_m; map.tvec_w_m]; map.vecPs2d_w_b]


q(1:3) = calib.rvec_b_c;
q(4:5) = calib.tvec_b_c(1:2);
q(6) = calib.dt;
q(7) = calib.k_odo_lin;
q(8) = calib.k_odo_rot;

idxStMk = flag.bCalibExt*5 + flag.bCalibTmp + flag.bCalibOdo*2;
q = zeros(idxStMk+6*mk.numMkId+3*odo.num,1);
q_init = [];
count = 1;
if flag.bCalibExt
    q_init_ext = [calib.rvec_b_c; calib.tvec_b_c(1:2)];
    q_init = [q_init; q_init_ext];
    vecrow_ext = count:count+4;
    count = count + 5;
end
if flag.bCalibTmp
    q_init_tmp = calib.dt;
    q_init = [q_init; q_init_tmp];
    vecrow_tmp = count;
    count = count + 1;
end
if flag.bCalibOdo
    q_init_odo = [calib.k_odo_lin; calib.k_odo_rot];
    q_init = [q_init; q_init_odo];
    vecrow_odo = count:count+1;
    count = count + 2;
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
options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
    'Display', 'iter-detailed', 'Jacobian', 'on', 'MaxIter', 50, 'ScaleProblem', 'Jacobian', 'TolX', 1e-6);
[q,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(...
    @(x)this.CostJointOpt5(x, mk, odo, time, calib, setting, flag), q, [], [], options);

% options.Algorithm = 'trust-region-reflective';
% [q,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(...
%     @(x)this.CostJointOpt5(x, mk, odo, time, calib, setting, flag), q, [], [], options);

%% save results

% refresh calib
if flag.bCalibExt
    q_ext = q(vecrow_ext);
    calib.rvec_b_c = q_ext(1:3);
    calib.tvec_b_c = [q_ext(4:5);0];
    calib.RefreshByVecbc;
end
if flag.bCalibTmp
    calib.dt = q(vecrow_tmp);
end
if flag.bCalibOdo
    q_odo = q(vecrow_odo);
    calib.k_odo_lin = q_odo(1);
    calib.k_odo_rot = q_odo(2);
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

