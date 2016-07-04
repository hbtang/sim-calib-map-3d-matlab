function [ T, R ] = FunVec2Trans3d( rvec, tvec )
%FUNVEC2TRANS3D Summary of this function goes here
%   Detailed explanation goes here
if size(tvec) == [1 3]
    tvec = tvec.';
end

R = rodrigues(rvec);
T = [R tvec; 0 0 0 1];

end

