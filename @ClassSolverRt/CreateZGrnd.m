function [mu_z, sigma_z, jacobian_z] = CreateZGrnd(this, ...
    mu_x, sigma_x, mu_y, sigma_y)
%FUNCVMGRNDPL % function: generate mean, cov, and measurement func, of virtual
% measurement z1 = h1(x,y) = x.'*y
% measurement z2 = h2(x,y) = x.'*x - 1

Z3 = zeros(3,3); I3 = eye(3);

%% for z1
% Hessian matrix of h(x,y)
H1 = [Z3 I3; I3 Z3];
% Covariance matrix of [x;y]
C1 = blkdiag(sigma_x, sigma_y);
% Jacobian matrix of h with [x;y]
J1 = [mu_y.' mu_x.'];
% generate z1
[mu_z1, sigma_z1] = FuncChiSqr(this, H1, C1);

%% for z2
% Hessian matrix of h(x,y)
H2 = [I3 Z3; Z3 Z3];
% Covariance matrix of [x;y]
C2 = C1;
% Jacobian matrix of h with [x;y]
J2 = [mu_x.' 0 0 0];
% generate z2
[mu_z2, sigma_z2] = FuncChiSqr(this, H2, C2);

%% output
mu_z = [mu_z1; mu_z2];
sigma_z = diag([sigma_z1;sigma_z2]);
jacobian_z = [J1; J2];

end

