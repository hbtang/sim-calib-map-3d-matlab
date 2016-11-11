function [vec_mu_z, mat_Sigma_z] = VirtualMeasure(this, mat_Sigma_x, mat_Sigma_y, cellmat_Hessian)
%VIRTUALMEASURE Summary of this function goes here
%   Detailed explanation goes here

vec_mu_z = zeros(numel(cellmat_Hessian), 1);
mat_Sigma_z = zeros(numel(cellmat_Hessian), numel(cellmat_Hessian));

C = blkdiag(mat_Sigma_x, mat_Sigma_y);
L = chol(C);

for j = 1:numel(cellmat_Hessian)
    H = cellmat_Hessian{j};
    A = H/2;
    
    [~,S,V] = svd(L*A*L.');
    R = V.';
    B = S;    
    sz = size(H);
    
    mu_z_j = 0;
    sigma_z_j = 0;
    for i = 1:sz(1)
        mu_z_j = mu_z_j + B(i,i);
        sigma_z_j = sigma_z_j + 2*B(i,i)^2;
    end
    
    vec_mu_z(j) = mu_z_j;
    mat_Sigma_z(j,j) = sigma_z_j;    
end

end

