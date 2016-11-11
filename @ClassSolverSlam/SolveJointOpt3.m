function SolveJointOpt3(this, measure, calib, map)
%SOLVESTEP2 solver step 2: solve all by joint optimization

%% init
disp(['Start full temporal-spatio calibration with joint optimization...']);

mk = measure.mk;
odo = measure.odo;
time = measure.time;

% define variable vector vec_q: 
% [rvec_b_c; x_b_c; y_b_c, dt_b_c; map.vecPt3d_w_m; map.vecPs2d_w_b]
q = zeros(6+3*mk.numMkId+3*odo.num,1);
q(1:3) = calib.rvec_b_c;
q(4:5) = calib.tvec_b_c(1:2);
q(6) = calib.dt;

idxStMk = 6;
for i = 1:mk.numMkId
    q(idxStMk+i*3-2:idxStMk+i*3) = map.mks.tvec_w_m(i,:).';
end
idxStOdo = idxStMk+3*mk.numMkId;
for i = 1:odo.num
    q(idxStOdo+3*i-2: idxStOdo+3*i) = map.kfs.ps2d_w_b(i,:).';
end

%% solve nonlinear least square, mapping and calibration
options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
    'Display', 'iter-detailed', 'Jacobian', 'on', 'MaxIter', 50, 'ScaleProblem', 'Jacobian', 'TolX', 1e-6);

[q,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(x)this.CostJointOpt3(x, mk, odo, time, calib), q, [], [], options);

%% save results
% refresh calib T3d_b_c
calib.rvec_b_c = q(1:3);
calib.tvec_b_c = [q(4:5);0];
calib.RefreshByVecbc;
calib.dt = q(6);

% refresh map
vecPt3d_w_m = zeros(mk.numMkId, 3);
vecPs2d_w_b = zeros(odo.num, 3);
for i = 1:mk.numMkId
    vecPt3d_w_m(i,:) = q(idxStMk+3*i-2:idxStMk+3*i).';
end
for i = 1:odo.num
    vecPs2d_w_b(i,:) = q(idxStOdo+3*i-2:idxStOdo+3*i).';
end
map.mks.tvec_w_m = vecPt3d_w_m;
map.kfs.ps2d_w_b = vecPs2d_w_b;
map.RefreshKfsByPs2dwb;

disp('Full temporal-spatio calibration with joint optimization done!');
disp(' ');

end

