function [ odo ] = FunRefineOdo( odo, k_dist, k_theta )
%PROC_REFINEODO Summary of this function goes here

%   Detailed explanation goes here
% k_theta = 1;
% k_dist = 1;
odo.ux = []; odo.uy = []; odo.w = [];
for i = 1:numel(odo.lp)-1
    x1 = odo.x(i); y1 = odo.y(i); theta1 = odo.theta(i);
    x2 = odo.x(i+1); y2 = odo.y(i+1); theta2 = odo.theta(i+1);     
    vec1 = [x1;y1;theta1];
    vec2 = [x2;y2;theta2];    
    T1 = vec_matrix_2d(vec1);
    T2 = vec_matrix_2d(vec2);
    T12 = inv(T1)*T2;    
    vec12 = vec_matrix_2d(T12);
    vec12(1:2) = vec12(1:2) * k_dist;
    vec12(3) = vec12(3) * k_theta;    
    odo.ux = [odo.ux; vec12(1)];
    odo.uy = [odo.uy; vec12(2)];
    odo.w = [odo.w; vec12(3)];
end
for i = 1:numel(odo.lp)-1
    x1 = odo.x(i); y1 = odo.y(i); theta1 = odo.theta(i);
    ux = odo.ux(i); uy = odo.uy(i); w = odo.w(i);    
    vec1 = [x1;y1;theta1];
    vec12 = [ux;uy;w];
    T1 = vec_matrix_2d(vec1);
    T12 = vec_matrix_2d(vec12);
    T2 = T1*T12;
    vec2 = vec_matrix_2d(T2);    
    odo.x(i+1) = vec2(1);
    odo.y(i+1) = vec2(2);
    odo.theta(i+1) = vec2(3);
end

end

