function [ mark ] = FunMkSeg( mark )
%PROCMKSEG segment mark observation as continues observation the same id

seg = [1 mark.num];
idLast = mark.id(1);
for i = 2:mark.num
    if mark.id(i) ~= idLast
        seg(end, 2) = i-1;
        seg(end+1, 1) = i;
        idLast = mark.id(i);
    end    
end
seg(end,2) = mark.num;
mark.seg = seg;
mark.numSeg = size(seg,1);
end

