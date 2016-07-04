function [ odo_out, mark_out ] = Pick_Odo( odo_in, mark_in )
%PICKODO Summary of this function goes here
%%   pick odometry measurements
%   offset larger than threshold, or mark detected
% dl_thresh = 300;
% dtheta_thresh = 10*pi/180;
dl_thresh = 50;
dtheta_thresh = 3*pi/180;

[sz_odo,tmp] = size(odo_in.stamp);
[sz_mark,tmp] = size(mark_in.stamp);
idx_mark = 1;
idx_odo_last = 1;

odo_out.stamp = [odo_in.stamp(1)];
odo_out.x = [odo_in.x(1)];
odo_out.y = [odo_in.y(1)];
odo_out.theta = [odo_in.theta(1)];

mark_in.odoIdx = [];

if mark_in.stamp(idx_mark) == odo_in.stamp(1)
    mark_in.odoIdx = [mark_in.odoIdx; 1];
    idx_mark = idx_mark + 1;
end

for i = (2:sz_odo)
    dx = odo_out.x(end) - odo_in.x(i);
    dy = odo_out.y(end) - odo_in.y(i);
    dtheta = odo_out.theta(end) - odo_in.theta(i);
    dl = sqrt(dx*dx + dy*dy);
    if (mark_in.stamp(idx_mark) == odo_in.stamp(i))
        odo_out.stamp = [odo_out.stamp; odo_in.stamp(i)];
        odo_out.x = [odo_out.x; odo_in.x(i)];
        odo_out.y = [odo_out.y; odo_in.y(i)];
        odo_out.theta = [odo_out.theta; odo_in.theta(i)];
        if idx_mark <= sz_mark
            sz_odo_out_now = size(odo_out.stamp);            
            mark_in.odoIdx = [mark_in.odoIdx; sz_odo_out_now(1)];
            if idx_mark ~= sz_mark
                idx_mark = idx_mark + 1;
            end
        end
    elseif (dl > dl_thresh) || (abs(dtheta) > dtheta_thresh)
        odo_out.stamp = [odo_out.stamp; odo_in.stamp(i)];
        odo_out.x = [odo_out.x; odo_in.x(i)];
        odo_out.y = [odo_out.y; odo_in.y(i)];
        odo_out.theta = [odo_out.theta; odo_in.theta(i)];
    end
end 
mark_out = mark_in;

end