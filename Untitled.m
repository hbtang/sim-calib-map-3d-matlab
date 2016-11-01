[C3, J3] = this.CostJointOpt3(q, mk, odo, time, calib);
figure;
plot(C3);

q2 = q; q2(6) = [];
[C2,J2] = this.CostJointOpt2(q2, mk, odo, calib);
figure;
plot(C2);

plot(C2-C3);

%%

dx = measure_raw.odo.x(2:end) - measure_raw.odo.x(1:end-1);
dy = measure_raw.odo.y(2:end) - measure_raw.odo.y(1:end-1);
dtheta = measure_raw.odo.theta(2:end) - measure_raw.odo.theta(1:end-1);

dt_odo = measure_raw.time.t_odo(2:end) - measure_raw.time.t_odo(1:end-1);
dt_mk = measure_raw.time.t_mk(2:end) - measure_raw.time.t_mk(1:end-1);

for i = 1:numel(dt_mk)
    dt_mk(i) = cnstr2period(dt_mk(i), 30, -30);
end

for i = 1:numel(dt_odo)
    dt_odo(i) = cnstr2period(dt_odo(i), 30, -30);
end

figure;
plot([dt_mk*1000 dx dy dtheta])

mean(dt_odo);

%%

mkId = 0;

rows = find(measure_raw.mk.id == mkId);
vec_lp = measure_raw.mk.lp(rows);

vec_mktvec = measure_raw.mk.tvec(rows,:);
vec_mk_x = vec_mktvec(:,1);
vec_mk_y = vec_mktvec(:,2);
vec_mk_z = vec_mktvec(:,3);

vec_t_mk = [];
for i = 1:numel(vec_lp)
    lp_i = vec_lp(i);
    row_time = find(measure_raw.time.lp == lp_i, 1);
    vec_t_mk = [vec_t_mk; measure_raw.time.t_mk(row_time)];
end

vec_dt = vec_t_mk(2:end) - vec_t_mk(1:end-1);
vec_dx = vec_mk_x(2:end) - vec_mk_x(1:end-1);

plot([vec_dt*1000 vec_dx])

%%

std(residual2(1:982*3))
std(residual3(1:982*3))
std(residual2(982*3+1:end))
std(residual3(982*3+1:end))









