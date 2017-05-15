function [ Jacobian ] = JacobianNum( FUN, x, delta )
%JACOBIANNUM to compute the Jacobian matrix by a numerical mathod

if nargin < 3
    delta = 1e-6;
end

y = FUN(x);
dim_x = numel(x);
dim_y = numel(y);
Jacobian = zeros(dim_y, dim_x);

for i = 1:dim_x
    x_bar = x;
    x_bar(i) = x_bar(i) + delta;
    
    y_bar = FUN(x_bar);
    dy = y_bar - y;

    Jacobian(:,i) = dy/delta;
end

