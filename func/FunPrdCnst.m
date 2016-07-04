function [ x_out ] = FunPrdCnst( x, x_max, x_min )
%FunPrdCnst Apply period constraint to variable x
%   Detailed explanation goes here
if x_max < x_min
    tmp = x_min;
    x_min = x_max;
    x_max = tmp;
end
period = x_max - x_min;
while x > x_max
    x = x - period;
end
while x <= x_min
    x = x + period;
end

x_out = x;
end

