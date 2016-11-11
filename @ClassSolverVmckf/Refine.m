function [mat_Sigma_x_ref] = Refine(this, ...
    flag, vec_mu_x, vec_mu_x_new, mat_Sigma_x_new)
%REFINE Summary of this function goes here
%   Detailed explanation goes here

% flag: 0 spatio, 1 spatio-temporal, 2 spatio-odo
vec_d_mu_x = vec_mu_x_new - vec_mu_x;

switch flag
    case 0
        % spatio only, mu_x = [q; x; y]
        mat_Sigma_r = mat_Sigma_x_new(1:4, 1:4);
        mat_Sigma_t = mat_Sigma_x_new(5:6, 5:6);
        
        dist_r = norm(vec_d_mu_x(1:4));
        eig_r = sqrt(eig(mat_Sigma_r));
        if dist_r > max(eig_r)*1.5 && dist_r > 0.03 && min(eig_r) < 3
            mat_Sigma_temp = (dist_r)^2*eye(4);
            mat_Sigma_x_new(1:4,1:4) = mat_Sigma_r + mat_Sigma_temp;
        end
        
        dist_t = norm(vec_d_mu_x(5:6));
        eig_t = sqrt(eig(mat_Sigma_t));
        if dist_t > max(eig_t)*3 && dist_t > 100 && min(eig_t) < 300
            mat_Sigma_temp = (dist_t)^2*eye(2);
            mat_Sigma_x_new(5:6,5:6) = mat_Sigma_x_new(5:6,5:6) + mat_Sigma_temp;
        end
        
        % low bound of sigma
        [U,S,V] = svd(mat_Sigma_x_new(1:4,1:4));
        S = max(S,eye(4)*1e-6);
        mat_Sigma_x_new(1:4,1:4) = U*S*V.';
        
        [U,S,V] = svd(mat_Sigma_x_new(5:6,5:6));
        S = max(S,eye(2)*1e-2);
        mat_Sigma_x_new(5:6,5:6) = U*S*V.';
        
        mat_Sigma_x_ref = mat_Sigma_x_new;
        
    case 1
    case 2
        % spatio-odo, mu_x = [r; x; y; k_lin; k_rot]
        
        mat_Sigma_r = mat_Sigma_x_new(1:3, 1:3);
        mat_Sigma_t = mat_Sigma_x_new(4:5, 4:5);
        mat_Sigma_k = mat_Sigma_x_new(6:7, 6:7);
        
        dist_r = norm(vec_d_mu_x(1:3));
        eig_r = sqrt(eig(mat_Sigma_r));
        if dist_r > max(eig_r)*1.5 && dist_r > 0.03 && min(eig_r) < 3
            mat_Sigma_temp = (dist_r)^2*eye(3);
            mat_Sigma_x_new(1:3,1:3) = mat_Sigma_r + mat_Sigma_temp;
        end
        
        dist_t = norm(vec_d_mu_x(4:5));
        eig_t = sqrt(eig(mat_Sigma_t));
        if dist_t > max(eig_t)*3 && dist_t > 100 && min(eig_t) < 300
            mat_Sigma_temp = (dist_t)^2*eye(2);
            mat_Sigma_x_new(4:5,4:5) = mat_Sigma_x_new(4:5,4:5) + mat_Sigma_temp;
        end
        
        % low bound of sigma
        [U,S,V] = svd(mat_Sigma_x_new(1:3,1:3));
        S = max(S,eye(3)*1e-6);
        mat_Sigma_x_new(1:3,1:3) = U*S*V.';
        
        [U,S,V] = svd(mat_Sigma_x_new(4:5,4:5));
        S = max(S,eye(2)*1e-2);
        mat_Sigma_x_new(4:5,4:5) = U*S*V.';
        
        [U,S,V] = svd(mat_Sigma_x_new(6:7,6:7));
        S = max(S,eye(2)*1e-6);
        mat_Sigma_x_new(6:7,6:7) = U*S*V.';
        
        mat_Sigma_x_ref = mat_Sigma_x_new; 
        
    otherwise
end


end

