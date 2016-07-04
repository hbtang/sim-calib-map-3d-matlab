function [ rvec, tvec ] = FunTrans2Vec3d( T )
%FUNTRANS2VEC3D Summary of this function goes here
%   Detailed explanation goes here

if size(T) ~= [4 4]
    error('dimension error!!!');
end

R = T(1:3, 1:3);
rvec = rodrigues(R);
tvec = T(1:3,4);

end

