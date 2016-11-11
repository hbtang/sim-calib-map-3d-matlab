function [ err_ret ] = Err_Local( measure, calib, bUseTime )
% Compute error of p_c_m between 2 consecutive instants

err_ret = [];

mk = measure.mk;
odo = measure.odo;
time = measure.time;

T3d_b_c = calib.T3d_b_c;

err_ret_debug = [];

vec_dx_mk = [];
vec_dx_odo = [];
vec_dx_odo_debug = [];
vec_dt_odo_debug = [];
vec_dt_mk_debug = [];

for i = 1:mk.numMkId
    mkId_i = mk.vecMkId(i);
    
    rows_mk = find(mk.id == mkId_i);
    vec_lp = mk.lp(rows_mk);
    
    for j = 1:numel(vec_lp)-1
        lp_1 = vec_lp(j);
        lp_2 = vec_lp(j+1);
        
        if abs(lp_2-lp_1) > 10
            continue;
        end
        
        row_mk_1 = rows_mk(j);
        row_mk_2 = rows_mk(j+1);
        row_odo_1 = find(odo.lp == lp_1, 1);
        row_odo_2 = find(odo.lp == lp_2, 1);
        row_time_1 = find(time.lp == lp_1, 1);
        row_time_2 = find(time.lp == lp_2, 1);
        
        ps2d_w_b1_odo = [odo.x(row_odo_1); odo.y(row_odo_1); odo.theta(row_odo_1)];
        v2d_w_b1_odo = [odo.vx(row_odo_1); odo.vy(row_odo_1); odo.vtheta(row_odo_1)];
        ps2d_w_b2_odo = [odo.x(row_odo_2); odo.y(row_odo_2); odo.theta(row_odo_2)];
        v2d_w_b2_odo = [odo.vx(row_odo_2); odo.vy(row_odo_2); odo.vtheta(row_odo_2)];
        
        p3d_c1_m_obs = mk.tvec(row_mk_1, :).';
        p3d_c2_m_obs = mk.tvec(row_mk_2, :).';
        
        if bUseTime
            dt1 = cnstr2period(time.t_mk(row_time_1)-time.t_odo(row_time_1)+calib.dt, 30, -30);
            dt2 = cnstr2period(time.t_mk(row_time_2)-time.t_odo(row_time_2)+calib.dt, 30, -30);
        else
            dt1 = 0;
            dt2 = 0;
        end
        
        ps2d_w_b1_dt = ps2d_w_b1_odo + dt1*v2d_w_b1_odo;
        ps2d_w_b2_dt = ps2d_w_b2_odo + dt2*v2d_w_b2_odo;
        
        ps2d_b1dt_b2dt = FunRelPos2d(ps2d_w_b1_dt, ps2d_w_b2_dt);
        rvec_b1dt_b2dt = [0;0;ps2d_b1dt_b2dt(3)];
        tvec_b1dt_b2dt = [ps2d_b1dt_b2dt(1:2);0];
        
        T3d_b1_b2 = FunVec2Trans3d( rvec_b1dt_b2dt, tvec_b1dt_b2dt );
        
        errTilde_b1_m = [p3d_c1_m_obs;1] - inv(T3d_b_c)*T3d_b1_b2*T3d_b_c*[p3d_c2_m_obs;1];
        err_p3d_b1_m = errTilde_b1_m(1:3);
        
        %Debug ...
        
        ps2d_b1_b2_debug = FunRelPos2d(ps2d_w_b1_odo, ps2d_w_b2_odo);
        rvec_b1_b2_debug = [0;0;ps2d_b1_b2_debug(3)];
        tvec_b1_b2_debug = [ps2d_b1_b2_debug(1:2);0];
        T3d_b1_b2_debug = FunVec2Trans3d( rvec_b1_b2_debug, tvec_b1_b2_debug );
        errTilde_b1_debug = [p3d_c1_m_obs;1] - inv(T3d_b_c)*T3d_b1_b2_debug*T3d_b_c*[p3d_c2_m_obs;1];
        err_p3d_b1_m_debug = errTilde_b1_debug(1:3);
        err_ret_debug = [err_ret_debug; err_p3d_b1_m_debug.'];
        
        dp_mk = p3d_c2_m_obs - p3d_c1_m_obs;
        vec_dx_mk = [vec_dx_mk; dp_mk(1)];
        vec_dx_odo = [vec_dx_odo; ps2d_b1dt_b2dt(1)];
        
        vec_dx_odo_debug = [vec_dx_odo_debug; ps2d_b1_b2_debug(1)];
        
        dt_b1b2_odo_debug = cnstr2period(time.t_odo(row_time_2)-time.t_odo(row_time_1), 30, -30);
        vec_dt_odo_debug = [vec_dt_odo_debug; dt_b1b2_odo_debug];
        
        dt_c1c2_mk_debug = cnstr2period(time.t_mk(row_time_2)-time.t_mk(row_time_1), 30, -30);
        vec_dt_mk_debug = [vec_dt_mk_debug; dt_c1c2_mk_debug];
        
        %Debug end
        
        err_ret = [err_ret; err_p3d_b1_m.'];
    end
end

%% debug
% stdErr = std(err_ret);
% stdErrDebug = std(err_ret_debug);
% plot([vec_dx_mk vec_dx_odo vec_dx_odo_debug])
% figure;
% plot([vec_dx_odo_debug 1000*vec_dt_odo_debug])
% figure;
% plot([vec_dx_mk 1000*vec_dt_mk_debug])

end

