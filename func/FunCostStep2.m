function [ F, J ] = FunCostStep2( vec_q, mark, odo, calib )
%FUNCOSTSTEP2 Summary of this function goes here
% F: 3*mark.num vector of projection error

%% init
T_cg_c = calib.T_cg_c;

F = zeros(3*mark.num,1);
pose2d_b_c = vec_q(1:3);
vec_pt3d_o_m = zeros(mark.numSeg,3);
for i = 1:mark.numSeg
    vec_pt3d_o_m(i,:) = vec_q(3*i+1: 3*i+3).';
end

rvec_b_cg = [0;0;pose2d_b_c(3)];
tvec_b_cg = [pose2d_b_c(1:2);0];
[T_b_cg] = FunVec2Trans3d(rvec_b_cg, tvec_b_cg);
T_b_c = T_b_cg*T_cg_c;

%% calculate cost function F for each observation
idSegNow = 1;
for i = 1:mark.num
    if i > mark.seg(idSegNow,2)
        idSegNow = idSegNow+1;
    end
    
    stamp = mark.stamp(i);
    x_o_b = odo.x(stamp);
    y_o_b = odo.y(stamp);
    theta_o_b = odo.theta(stamp);
    rvec_o_b = [0; 0; theta_o_b];
    tvec_o_b = [x_o_b; y_o_b; 0];
    T_o_b = FunVec2Trans3d(rvec_o_b, tvec_o_b);
    
    tvec_c_m = mark.tvec(i,:).';
    tvec_o_m_model = vec_pt3d_o_m(idSegNow,:).';
    
    tvec_o_m = T_o_b*T_b_c*[tvec_c_m;1];
    tvec_o_m = tvec_o_m(1:3);
    
    %     tvec_c_m_model = inv(T_o_b*T_b_c)*[tvec_o_m_model;1];
    %     tvec_c_m_model = tvec_c_m_model(1:3);
    
    F(3*i-2: 3*i) = tvec_o_m_model - tvec_o_m;
end

%% calculate Jacobian of F

if nargout > 1
    J = zeros(3*mark.num, numel(vec_q));
    idSegNow = 1;
    for i = 1:mark.num
        if i > mark.seg(idSegNow,2)
            idSegNow = idSegNow+1;
        end
        
        stamp = mark.stamp(i);
        theta_o_b = odo.theta(stamp);
        tvec_c_m = mark.tvec(i,:).';
        tvec_cg_m = T_cg_c(1:3, 1:3)*tvec_c_m + T_cg_c(1:3, 4);
        
        J1 = zeros(3,3);
        theta_o_cg = theta_o_b + vec_q(3);
        R2d_o_cg = [cos(theta_o_cg) -sin(theta_o_cg); sin(theta_o_cg) cos(theta_o_cg)];
        J1(1:2, 3) = -R2d_o_cg*[-tvec_cg_m(2); tvec_cg_m(1)];        
        J1(1:2, 1:2) = -[cos(theta_o_b) -sin(theta_o_b);...
            sin(theta_o_b) cos(theta_o_b)];
        
        J(3*i-2:3*i, 1:3) = J(3*i-2:3*i, 1:3) + J1;        
        J(3*i-2:3*i, 3*idSegNow+1: 3*idSegNow+3) = ...
            J(3*i-2:3*i, 3*idSegNow+1: 3*idSegNow+3) + eye(3);
        
    end
end
end

