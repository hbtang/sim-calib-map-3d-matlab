function [ time_ret ] = TimeOdo_ConstVel( time_in, odo_in )
% Create new time data, assuming robot velocity constant during local
% period

vec_dt = time_in(2:end) - time_in(1:end-1);
for i = 1:numel(vec_dt)
    vec_dt(i) = cnstr2period(vec_dt(i), 30, -30);
end
dt_all = sum(vec_dt);
dt_mean = mean(vec_dt);

vec_dx = odo_in.x(2:end) - odo_in.x(1:end-1);
vec_dy = odo_in.y(2:end) - odo_in.y(1:end-1);
vec_dl = sqrt(vec_dx.*vec_dx+vec_dy.*vec_dy);
vec_dl = max(vec_dl, 1e-6);

vec_dl_smooth = smooth(vec_dl, 10);
vec_k = vec_dl./vec_dl_smooth;
for i = 1:numel(vec_k)
    if vec_k(i) > 3 || vec_k(i) < 0.05
        vec_k(i) = 1;
    end
end

k_mean = mean(vec_k);
vec_dt_new = (dt_mean/k_mean)*vec_k;

time_ret = time_in(1);
for i = 2:numel(time_in)
    time_ret(i,1) = time_ret(i-1,1) + vec_dt_new(i-1);
    time_ret(i,1) = cnstr2period(time_ret(i,1), 0, 60);
end

end

