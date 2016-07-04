function [ T3d ] = FunPs2d2T3d( ps2d )
%FUNPS2D2T3D obtain 3d transformation matrix from 2d pose
rvec = [0;0;ps2d(3)];
tvec = [ps2d(1);ps2d(2);0];
R3d = rodrigues(rvec);
T3d = [R3d tvec; 0 0 0 1];

end

