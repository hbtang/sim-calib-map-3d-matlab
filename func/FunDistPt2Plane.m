function [ dist ] = FunDistPt2Plane( vGrnd, vPt )
%FUNDISTPT2PLANE obtain distance from a 3d point to a 3d plane
% vGrnd, 4*1 array, plane vector
% vPt, 3*1 array, point vector

if size(vGrnd) ~= [4 1]
    error('Error, dimension does not fit!');
end
if size(vPt) ~= [3 1]
    error('Error, dimension does not fit!');
end

dist = vGrnd.'*[vPt;1]/norm(vGrnd(1:3));

end

