function odo_out = NoiseOdo(this, odo_true, calib, setting)
%NOISEODO create odo with noise and dt

timestep = this.timestep;
error = setting.error;
dt_b_c = calib.dt;

%% add time delay
odo_true_time = odo_true;
for i = 1:numel(odo_true.lp)
    %% compute v
    if i == 1 || (dt_b_c < 0 && i ~= numel(odo_true.lp))
        i1 = i;
        i2 = i+1;
    else
        i1 = i-1;
        i2 = i;
    end
    vx = (odo_true.x(i2) - odo_true.x(i1))/timestep;
    vy = (odo_true.y(i2) - odo_true.y(i1))/timestep;
    dtheta = odo_true.theta(i2) - odo_true.theta(i1);
    dtheta = cnstr2period(dtheta, pi, -pi);
    vtheta = dtheta/timestep;
    
    %% compute odo_true_time with delay
    odo_true_time.x(i) = -dt_b_c*vx + odo_true.x(i);
    odo_true_time.y(i) = -dt_b_c*vy + odo_true.y(i);
    odo_true_time.theta(i) = -dt_b_c*vtheta + odo_true.theta(i);
    odo_true_time.theta(i) = cnstr2period(odo_true_time.theta(i), pi, -pi);
end

%% add noise
odo_out = struct('lp',odo_true_time.lp(1), 'x',odo_true_time.x(1),'y',odo_true_time.y(1),'theta',odo_true_time.theta(1));
stdratio_lin = error.odo.stdratio_lin;
stdratio_rot = error.odo.stdratio_rot;
num_odo = numel(odo_true_time.lp);
k_odo_lin = calib.k_odo_lin;
k_odo_rot = calib.k_odo_rot;
for i = 2:num_odo
    lp = odo_true_time.lp(i);
    
    ps2d_o_b1 = [odo_true_time.x(i-1);odo_true_time.y(i-1);odo_true_time.theta(i-1)];
    ps2d_o_b2 = [odo_true_time.x(i);odo_true_time.y(i);odo_true_time.theta(i)];
    ps2d_b1_b2 = FunRelPos2d(ps2d_o_b1, ps2d_o_b2);
    ps2d_b1_b2(1:2) = k_odo_lin*ps2d_b1_b2(1:2);
    ps2d_b1_b2(3) = k_odo_rot*ps2d_b1_b2(3);
            
    stdErrLin = stdratio_lin*norm(ps2d_b1_b2(1:2));
    stdErrRot = stdratio_rot*abs(ps2d_b1_b2(3));
    ps2d_b1_b2 = normrnd(ps2d_b1_b2, [stdErrLin;stdErrLin;stdErrRot]);
    
    ps2d_oNs_b1 = [odo_out.x(i-1);odo_out.y(i-1);odo_out.theta(i-1)];
    ps2d_oNs_b2 = FunMove2d(ps2d_oNs_b1, ps2d_b1_b2);
    
    odo_out.x(i) = ps2d_oNs_b2(1);
    odo_out.y(i) = ps2d_oNs_b2(2);
    odo_out.theta(i) = ps2d_oNs_b2(3);
    odo_out.lp(i) = lp;
end