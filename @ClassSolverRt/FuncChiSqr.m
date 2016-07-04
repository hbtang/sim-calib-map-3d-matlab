function [vecMu_z, matSigma_z] = FuncChiSqr(this, H, C)
%FUNCCHISQR obtain z according to chi-square, by Hessian H and Covariance C

L = chol(C);
A = H/2;

[~,S,V] = svd(L*A*L.');
R = V.';
B = S;

sz = size(H);

vecMu_z = 0;
matSigma_z = 0;
for i = 1:sz(1)
    vecMu_z = vecMu_z + B(i,i);
    matSigma_z = matSigma_z + 2*B(i,i)^2;
end

end

