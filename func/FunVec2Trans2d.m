function [ T2d ] = FunVec2Trans2d( vec2d )
%FUNVEC2TRANS2D Summary of this function goes here
%   Detailed explanation goes here

T2d = [cos(vec2d(3)) -sin(vec2d(3)) vec2d(1);...
    sin(vec2d(3)) cos(vec2d(3)) vec2d(2);...
    0 0 1];

end

