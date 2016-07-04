function SolveJointOpt(this, measure, calib, map)
%SOLVESTEP2 solver step 2: solve all by joint optimization

%% init
disp(['Start full calibration with joint optimization...']);

mk = measure.mk;
odo = measure.odo;
ps2d_b_cg = calib.GetPs2dbcg;

% define variable vector vec_q: [ps2d_b_cg;map.vecPt3d_w_m;map.vecPs2d_w_b]
q = zeros(3+3*mk.numMkId+3*odo.num,1);
q(1:3) = ps2d_b_cg;
for i = 1:mk.numMkId
    q(i*3+1:i*3+3) = map.mks.tvec_w_m(i,:).';
end
for i = 1:odo.num
    idxSt = 3+3*mk.numMkId;
    q(idxSt+3*i-2: idxSt+3*i) = map.kfs.ps2d_w_b(i,:).';
end

%% solve nonlinear least square, mapping and calibration
options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
    'Display', 'iter-detailed', 'Jacobian', 'on', 'MaxIter',50, 'ScaleProblem', 'Jacobian', 'TolX', 1e-6);

[q,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(@(x)this.CostJointOpt(x, mk, odo, calib), q, [], [], options);

%% save results
% parse vec_q
ps2d_b_cg = q(1:3);
vecPt3d_w_m = zeros(mk.numMkId, 3);
vecPs2d_w_b = zeros(odo.num, 3);
for i = 1:mk.numMkId
    vecPt3d_w_m(i,:) = q(3+3*i-2:3+3*i).';
end
for i = 1:odo.num
    id_tmp = 3+3*mk.numMkId;
    vecPs2d_w_b(i,:) = q(id_tmp+3*i-2:id_tmp+3*i).';
end
calib.SetPs2dbcg(ps2d_b_cg);
map.mks.tvec_w_m = vecPt3d_w_m;
map.kfs.ps2d_w_b = vecPs2d_w_b;
map.RefreshKfsByPs2dwb;

disp('Full calibration with joint optimization done!');
disp(' ');

end

