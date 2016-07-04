function [ vec2d ] = FunTrans2Vec2d( T2d )
%FUNTRANS2VEC2D Summary of this function goes here
%   Detailed explanation goes here
vec2d = zeros(3,1);
vec2d(1) = T2d(1,3);
vec2d(2) = T2d(2,3);
vec2d(3) = atan2(T2d(2,1), T2d(1,1));

end

