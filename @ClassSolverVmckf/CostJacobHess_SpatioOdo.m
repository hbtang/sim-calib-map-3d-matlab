function [vec_cost, mat_Jacobian, cellmat_Hessian] = ...
    CostJacobHess_SpatioOdo(this, vec_mu_x, vec_mu_y)
%COSTAPPROXCNSTR to compute the quadratic approximation of constraint.

%% compute cost function
vec_cost = Cost_SpatioOdo(this, vec_mu_x, vec_mu_y);

%% compute Jacobian and Hessian
if nargout > 1
    
    %% compute Jacobian numerically
    delta = 1e-3;
    
    num_x = numel(vec_mu_x);
    num_y = numel(vec_mu_y);
    num_c = numel(vec_cost);
    
    cellmat_cost_dx = cell(num_x, 2);
    cellmat_cost_dy = cell(num_y, 2);
    mat_Jacobian_x = zeros(num_c, num_x);
    mat_Jacobian_y = zeros(num_c, num_y);
    
    for i = 1:num_x
        vec_mu_dx_1 = vec_mu_x;
        vec_mu_dx_1(i) = vec_mu_dx_1(i) + delta;
        vec_mu_dx_2 = vec_mu_x;
        vec_mu_dx_2(i) = vec_mu_dx_2(i) - delta;
        
        vec_cost_1 = Cost_SpatioOdo(this, vec_mu_dx_1, vec_mu_y);
        vec_cost_2 = Cost_SpatioOdo(this, vec_mu_dx_2, vec_mu_y);
        cellmat_cost_dx(i,:) = {vec_cost_1 vec_cost_2};
        
        mat_Jacobian_x(:,i) = (vec_cost_1-vec_cost_2)/(2*delta);
    end
    
    for i = 1:num_y
        vec_mu_dy_1 = vec_mu_y;
        vec_mu_dy_1(i) = vec_mu_dy_1(i) + delta;
        vec_mu_dy_2 = vec_mu_y;
        vec_mu_dy_2(i) = vec_mu_dy_2(i) - delta;
        
        vec_cost_1 = Cost_SpatioOdo(this, vec_mu_x, vec_mu_dy_1);
        vec_cost_2 = Cost_SpatioOdo(this, vec_mu_x, vec_mu_dy_2);
        cellmat_cost_dy(i,:) = {vec_cost_1 vec_cost_2};
        
        mat_Jacobian_y(:,i) = (vec_cost_1-vec_cost_2)/(2*delta);
    end
    
    cellmat_cost_d = [cellmat_cost_dx; cellmat_cost_dy];
    mat_Jacobian = [mat_Jacobian_x mat_Jacobian_y];
    
    %% compute Hessian numerically
    cellmat_Hessian = cell(num_c, 1);
    for i = 1:num_c
        cellmat_Hessian{i} = zeros(num_x+num_y, num_x+num_y);
    end
    
    mat_Hessian_bool = eye(num_x+num_y, num_x+num_y);
    
    vecrow_rvec_b_c = [1;2;3];
    vecrow_tvec_b_c = [4;5];
    vecrow_k_odo_lin = 6;
    vecrow_k_odo_rot = 7;
    vecrow_tvec_b1_b2 = [8;9];
    vecrow_rvec_b1_b2 = 10;
    vecrow_tvec_c1_m = [11;12;13];
    vecrow_tvec_c2_m = [14;15;16];
    
    vecrow_1 = [vecrow_rvec_b_c;vecrow_tvec_c1_m];
    vecrow_2 = [vecrow_rvec_b_c; vecrow_k_odo_rot; vecrow_rvec_b1_b2; vecrow_tvec_c2_m];
    vecrow_3 = [vecrow_k_odo_rot; vecrow_rvec_b1_b2; vecrow_tvec_b1_b2];
    vecrow_4 = [vecrow_k_odo_lin; vecrow_tvec_b1_b2];
    
    mat_Hessian_bool(vecrow_1, vecrow_1) = ones(numel(vecrow_1));
    mat_Hessian_bool(vecrow_2, vecrow_2) = ones(numel(vecrow_2));
    mat_Hessian_bool(vecrow_3, vecrow_3) = ones(numel(vecrow_3));
    mat_Hessian_bool(vecrow_4, vecrow_4) = ones(numel(vecrow_4));
    
    cellmat_cost_dd = cell(num_x+num_y, num_x+num_y);
    
    vec_mu_xy = [vec_mu_x; vec_mu_y];
    
    % off-diagonal elements
    for i = 1:(num_x+num_y-1)
        for j = i+1:(num_x+num_y)
            if mat_Hessian_bool(i,j) == 1
                
                vec_mu_xy_d = vec_mu_xy;
                vec_mu_xy_d(i) = vec_mu_xy_d(i)+delta;
                vec_mu_xy_d(j) = vec_mu_xy_d(j)+delta;
                
                vec_mu_x_d = vec_mu_xy_d(1:num_x);
                vec_mu_y_d = vec_mu_xy_d(num_x+1:num_x+num_y);
                
                vec_cost_d = Cost_SpatioOdo(this, vec_mu_x_d, vec_mu_y_d);
                cellmat_cost_dd{i,j} = vec_cost_d;
                cellmat_cost_dd{j,i} = vec_cost_d;
                
                for k = 1:num_c
                    
                    cellmat_Hessian{k}(i,j) = ...
                        (cellmat_cost_dd{i,j}(k)+vec_cost(k)-cellmat_cost_d{i,1}(k)-cellmat_cost_d{j,1}(k))/(delta*delta);
                    cellmat_Hessian{k}(j,i) = cellmat_Hessian{k}(i,j);
                    
                end
            end
        end
    end
    
    % diagonal elements
    for i = 1:(num_x+num_y-1)        
        if mat_Hessian_bool(i,i) == 1                        
            for k = 1:num_c                
                cellmat_Hessian{k}(i,i) = ...
                    (cellmat_cost_d{i,1}(k)+cellmat_cost_d{i,2}(k)-2*vec_cost(k))/(delta*delta);                
            end
        end
    end
    
end

end

