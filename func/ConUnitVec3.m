function [ c, ceq ] = ConUnitVec3( q )
%CONUNITVEC3 Summary of this function goes here
%   Detailed explanation goes here
c = [];
ceq = q(1:3).'*q(1:3) - 1;

end

