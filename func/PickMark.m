function [ mark_out ] = Pick_Mark( mark_in )
%PICK_MARK Summary of this function goes here
%   Detailed explanation goes here
dl_thresh = 50;
dtheta_thresh = 3*pi/180;

sz_mark = size(mark_in.stamp);
sz_mark = sz_mark(1);

mark_out.stamp = mark_in.stamp(1);
mark_out.id = mark_in.id(1);
mark_out.vec2d = mark_in.vec2d(1,:);



for i = (2:sz_mark)
    ifPickMark = false;
    if mark_in.id(i) ~= mark_out.id(end)
        ifPickMark = true;
    else
        vec_now = mark_in.vec2d(i,:)';
        vec_last = mark_out.vec2d(end,:)';        
        dl = norm(vec_now(1:2)-vec_last(1:2));
        dtheta = vec_now(3)-vec_now(3);
        if dl > dl_thresh
            ifPickMark = true;
        end
        if abs(dtheta) > dtheta_thresh
            ifPickMark = true;
        end
    end
    
    if ifPickMark
        mark_out.stamp = [mark_out.stamp;mark_in.stamp(i)];
        mark_out.id = [mark_out.id;mark_in.id(i)];
        mark_out.vec2d = [mark_out.vec2d;mark_in.vec2d(i,:)];        
    end   
end


end

