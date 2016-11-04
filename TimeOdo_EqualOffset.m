function [ time_out ] = TimeOdo_EqualOffset( time_in )
% to create new time data, with average offsets between frames

t_st = time_in(1);
time_out = t_st;

vec_dt = time_in(2:end) - time_in(1:end-1);
for i = 1:numel(vec_dt)
    vec_dt(i) = cnstr2period(vec_dt(i), 30, -30);
end
dt_mean = mean(vec_dt);

for i = 2:numel(time_in)
    time_out(i,1) = time_out(i-1,1) + dt_mean;
    time_out(i,1) = cnstr2period(time_out(i,1), 0, 60);
end

end

