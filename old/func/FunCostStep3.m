function [ F, J ] = FunCostStep3( vec_q, mark, odo, calib )
%FUNCOSTSTEP2 Summary of this function goes here
% F: 3*mark.num + 3*odo.num vector of projection error
F = zeros(3*mark.num + 3*odo.num,1);

%% parse vec_q
ps2d_b_cg = vec_q(1:3);
vecPt3d_w_m = zeros(mark.numId, 3);
vecPs2d_w_b = zeros(odo.num, 3);
for i = 1:mark.numId
    vecPt3d_w_m(i,:) = vec_q(3+3*i-2:3+3*i).';
end
for i = 1:odo.num
    id_tmp = 3+3*mark.numId;
    vecPs2d_w_b(i,:) = vec_q(id_tmp+3*i-2:id_tmp+3*i).';
end

%% calculate T_b_c
T_cg_c = calib.T_cg_c;
rvec_b_cg = [0;0;ps2d_b_cg(3)];
tvec_b_cg = [ps2d_b_cg(1:2);0];
[T_b_cg] = FunVec2Trans3d(rvec_b_cg, tvec_b_cg);
T_b_c = T_b_cg*T_cg_c;

%% calculate cost function F
% mark observation part

for i = 1:mark.num
    stamp = mark.stamp(i);
    x_w_b = vecPs2d_w_b(stamp,1);
    y_w_b = vecPs2d_w_b(stamp,2);
    theta_w_b = vecPs2d_w_b(stamp,3);
    rvec_w_b = [0; 0; theta_w_b];
    tvec_w_b = [x_w_b; y_w_b; 0];
    T_w_b = FunVec2Trans3d(rvec_w_b, tvec_w_b);
    
    tvec_c_m = mark.tvec(i,:).';
    idMk = mark.id(i);
    idMkOrd = find(mark.mkHash(:,1) == idMk, 1);
    tvec_w_m_model = vecPt3d_w_m(idMkOrd,:).';
    
    T_w_c = T_w_b*T_b_c;
    tvec_w_m = T_w_c(1:3,1:3)*tvec_c_m + T_w_c(1:3,4);
    
    dist = norm(tvec_c_m);
    std_z_c2 = max(dist*0.05, 10);
    std_x_c2 = max(dist*0.01, 1);
    std_y_c2 = max(dist*0.01, 1);
    mat_std_c2 = diag([std_x_c2;std_y_c2;std_z_c2]);
    R3d_c_w = inv(T_w_c(1:3,1:3));
    R3d_c_c2 = FunRotPt2ZAxle(tvec_c_m);
    R3d_c2_w = inv(R3d_c_c2)*R3d_c_w;
    mat_std = mat_std_c2*R3d_c2_w;
    
%     mat_std = diag([dist*0.01;dist*0.01;dist*0.01]);
    mats_std_mk{i} = mat_std;
    
    F(3*i-2: 3*i) = inv(mat_std)*(tvec_w_m_model-tvec_w_m);
end

% odometry part
idStTmp = 3*mark.num;
for i = 1:odo.num
    if i == 1
        x_w_b1 = 0; y_w_b1 = 0; theta_w_b1 = 0;
        ps2d_b1_b2_odo = [0;0;0];
    else
        x_w_b1 = vecPs2d_w_b(i-1,1);
        y_w_b1 = vecPs2d_w_b(i-1,2);
        theta_w_b1 = vecPs2d_w_b(i-1,3);
        ps2d_b1_b2_odo = FunRelPos2d([odo.x(i-1);odo.y(i-1);odo.theta(i-1)], ...
            [odo.x(i);odo.y(i);odo.theta(i)]);
    end
    
    x_w_b2 = vecPs2d_w_b(i,1);
    y_w_b2 = vecPs2d_w_b(i,2);
    theta_w_b2 = vecPs2d_w_b(i,3);
    
    ps2d_w_b1 = [x_w_b1; y_w_b1; theta_w_b1];
    ps2d_w_b2 = [x_w_b2; y_w_b2; theta_w_b2];
    ps2d_b1_b2_mod = FunRelPos2d(ps2d_w_b1, ps2d_w_b2);
    FunPrdCnst(ps2d_b1_b2_mod(3), ps2d_b1_b2_odo+pi, ps2d_b1_b2_odo-pi);
    
    std_trans = max(norm(ps2d_b1_b2_odo(1:2))*0.03, 1);
    std_rot = max(abs(ps2d_b1_b2_odo(3))*0.03, 0.1*pi/180);    
    mat_std = diag([std_trans;std_trans;std_rot]);
    mats_std_odo{i} = mat_std;
    F(idStTmp+3*i-2:idStTmp+3*i) = inv(mat_std)*(ps2d_b1_b2_mod-ps2d_b1_b2_odo);
end

%% calculate Jacobian of F
if nargout > 1
    % allocate J
    J = zeros(3*mark.num+3*odo.num, numel(vec_q));
    
    % calculate J of mark observation
    for i = 1:mark.num        
        stamp = mark.stamp(i);
        idMk = mark.id(i);
        idMkOrd = find(mark.mkHash(:,1) == idMk, 1);
        
        theta_w_b = vecPs2d_w_b(stamp,3);
        tvec_c_m = mark.tvec(i,:).';
        tvec_cg_m = T_cg_c(1:3, 1:3)*tvec_c_m + T_cg_c(1:3, 4);
        
        mat_std = mats_std_mk{i};
        
        % Jacoian on camera extrinsic ps2d_b_cg
        J1 = zeros(3,3);
        theta_w_cg = theta_w_b + vec_q(3);
        R2d_w_cg = [cos(theta_w_cg) -sin(theta_w_cg); sin(theta_w_cg) cos(theta_w_cg)];
        J1(1:2, 3) = -R2d_w_cg*[-tvec_cg_m(2); tvec_cg_m(1)];
        J1(1:2, 1:2) = -[cos(theta_w_b) -sin(theta_w_b);...
            sin(theta_w_b) cos(theta_w_b)];        
        J(3*i-2:3*i, 1:3) = J(3*i-2:3*i, 1:3) + inv(mat_std)*J1;
        
        % Jacobian on marker position pt3d_w_m
        J(3*i-2:3*i, 3*idMkOrd+1:3*idMkOrd+3) = ...
            J(3*i-2:3*i, 3*idMkOrd+1: 3*idMkOrd+3) + inv(mat_std)*eye(3); 
        
        %Jacobian on robot pose ps2d_w_b
        tvec_b_m = T_b_c(1:3,1:3)*tvec_c_m + T_b_c(1:3,4);
        R2d_w_b = [cos(theta_w_b) -sin(theta_w_b); sin(theta_w_b) cos(theta_w_b)];
        J3 = zeros(3,3);
        J3(1:2,3) = -R2d_w_b*[-tvec_b_m(2); tvec_b_m(1)];
        J3(1:2,1:2) = -eye(2);        
        J(3*i-2:3*i, 3+3*mark.numId+3*stamp-2:3+3*mark.numId+3*stamp) = inv(mat_std)*J3;
    end
    
    % calculate J of odometry
    row_st = 3*mark.num;
    col_st = 3+3*mark.numId;
    mat_std = mats_std_odo{1};
    J(row_st+1:row_st+3, col_st+1:col_st+3) = inv(mat_std);    
    for i = 2:odo.num
        ps_w_b1 = vecPs2d_w_b(i-1,:);
        ps_w_b2 = vecPs2d_w_b(i,:);        
        [J1, J2] = FunJacobianRelPs2d(ps_w_b1, ps_w_b2);
        mat_std = mats_std_odo{i};
        J(row_st+3*i-2:row_st+3*i, col_st+3*i-5:col_st+3*i-3) = inv(mat_std)*J1;
        J(row_st+3*i-2:row_st+3*i, col_st+3*i-2:col_st+3*i) = inv(mat_std)*J2;        
    end
end
end

