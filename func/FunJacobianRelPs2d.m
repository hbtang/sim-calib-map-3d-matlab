function [ J1, J2 ] = FunJacobianRelPs2d( ps_1, ps_2 )
%FUNJACOBIANRELPS2D calculate Jacobian matrix of relative pose regard to
% pose_1 and pose 2

x1 = ps_1(1); y1 = ps_1(2); theta1 = ps_1(3);
x2 = ps_2(1); y2 = ps_2(2); theta2 = ps_2(3);

J1 = [-cos(theta1), -sin(theta1), y2*cos(theta1) - y1*cos(theta1) + x1*sin(theta1) - x2*sin(theta1);...
    sin(theta1), -cos(theta1), x1*cos(theta1) - x2*cos(theta1) + y1*sin(theta1) - y2*sin(theta1);...
    0 0 -1];

J2 = [cos(theta1), sin(theta1), 0;...
    -sin(theta1), cos(theta1), 0;...
    0 0 1];
end

