function [ err_Mk, err_Odo, err_MkNormal, err_OdoNormal ] = Err_Slam( measure, calib, map, bUseTime, errConfig )
% Compute error vector of both mark measurements and odometry measurement

%% init data
mk = measure.mk;
odo = measure.odo;
time = measure.time;

vecPs2d_w_b = map.kfs.ps2d_w_b;

T3d_b_c = calib.T3d_b_c;
dt_b_c = calib.dt;
k_odo_lin = calib.k_odo_lin;
k_odo_rot = calib.k_odo_rot;

err_Mk = zeros(mk.num, 3);
err_MkNormal = zeros(mk.num, 3);
err_Odo = zeros(odo.num-1, 3);
err_OdoNormal = zeros(odo.num-1, 3);

vecDt = zeros(numel(time.lp), 1);
for i = 1:numel(time.lp)
    dtTmp = cnstr2period(time.t_mk(i) - time.t_odo(i), 30, -30);
    vecDt(i,1) = dtTmp;
end

%% mark observation part
for i = 1:mk.num
    lp = mk.lp(i);
    row_odo = find(odo.lp == lp,1);
    x_w_b = vecPs2d_w_b(row_odo,1);
    y_w_b = vecPs2d_w_b(row_odo,2);
    theta_w_b = vecPs2d_w_b(row_odo,3);
    rvec_w_b = [0; 0; theta_w_b];
    tvec_w_b = [x_w_b; y_w_b; 0];
    T3d_w_b = FunVec2Trans3d(rvec_w_b, tvec_w_b);
    
    tvec_c_m = mk.tvec(i,:).';
    mkId = mk.id(i);
    row_mapMk = find(map.mks.id(:,1) == mkId, 1);
    tvec_w_m = map.mks.tvec_w_m(row_mapMk,:).';
    
    err_tvec_c_m = [tvec_c_m; 1] - (T3d_w_b*T3d_b_c)\[tvec_w_m; 1];
    err_tvec_c_m = err_tvec_c_m(1:3);
    
    err_Mk(i,:) = err_tvec_c_m.';
    err_MkNormal(i,:) = (sqrt(CovMk( tvec_c_m, errConfig ))\err_tvec_c_m).';
end

%% odometry part
for i = 2:odo.num    
    if bUseTime
        dt_b1_c1 = (vecDt(i-1) + dt_b_c);
        dt_b2_c2 = (vecDt(i) + dt_b_c);
    else
        dt_b1_c1 = 0;
        dt_b2_c2 = 0;
    end
    
    ps2d_w_b1_odo = [odo.x(i-1);odo.y(i-1);odo.theta(i-1)];
    ps2d_w_b1t_odo = ps2d_w_b1_odo + ...
        dt_b1_c1*[odo.vx(i-1);odo.vy(i-1);odo.vtheta(i-1)];
    ps2d_w_b2_odo = [odo.x(i);odo.y(i);odo.theta(i)];
    ps2d_w_b2t_odo = ps2d_w_b2_odo + ...
        dt_b2_c2*[odo.vx(i);odo.vy(i);odo.vtheta(i)];
    
    ps2d_b1_b2_odo = FunRelPos2d(ps2d_w_b1t_odo, ps2d_w_b2t_odo);
    ps2d_b1_b2_odo(1:2) = k_odo_lin*ps2d_b1_b2_odo(1:2);
    ps2d_b1_b2_odo(3) = k_odo_rot*ps2d_b1_b2_odo(3);
    
    x_w_b1 = vecPs2d_w_b(i-1,1);
    y_w_b1 = vecPs2d_w_b(i-1,2);
    theta_w_b1 = vecPs2d_w_b(i-1,3);
    
    x_w_b2 = vecPs2d_w_b(i,1);
    y_w_b2 = vecPs2d_w_b(i,2);
    theta_w_b2 = vecPs2d_w_b(i,3);
    
    ps2d_w_b1 = [x_w_b1; y_w_b1; theta_w_b1];
    ps2d_w_b2 = [x_w_b2; y_w_b2; theta_w_b2];
    ps2d_b1_b2_map = FunRelPos2d(ps2d_w_b1, ps2d_w_b2);
    ps2d_b1_b2_map(3) = FunPrdCnst(ps2d_b1_b2_map(3), ps2d_b1_b2_odo+pi, ps2d_b1_b2_odo-pi);
    
    errPs2d_b1_b2 = ps2d_b1_b2_odo - ps2d_b1_b2_map;
    
    err_Odo(i-1,:) = errPs2d_b1_b2.';
    err_OdoNormal(i-1,:) = (sqrt(CovOdo( ps2d_b1_b2_odo, errConfig ))\errPs2d_b1_b2).';
    
    if abs(err_OdoNormal(i-1,3)) > 20
        debug = 1;
    end
end


end

