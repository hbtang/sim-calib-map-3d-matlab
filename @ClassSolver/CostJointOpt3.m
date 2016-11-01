function [ vecCost, matJacobian ] = CostJointOpt3( this, q, mk, odo, time, calib )
%COSTSTEP2

stdErrRatioMkX = this.errConfig.stdErrRatioMkX;
stdErrRatioMkY = this.errConfig.stdErrRatioMkY;
stdErrRatioMkZ = this.errConfig.stdErrRatioMkZ;
stdErrRatioOdoLin = this.errConfig.stdErrRatioOdoLin;
stdErrRatioOdoRot = this.errConfig.stdErrRatioOdoRot;

% vecCost: 3*mk.num + 3*odo.num vector of projection error
vecCost = zeros(3*mk.num + 3*odo.num,1);

%% parse q: rvec_b_c(1:3), tvec_b_c(1:2), vecPt3d_w_m, vecPs2d_w_b

rvec_b_c = q(1:3);
tvec_b_c = [q(4:5);0];
[ T3d_b_c, R3d_b_c ] = FunVec2Trans3d( rvec_b_c, tvec_b_c );

vecPt3d_w_m = zeros(mk.numMkId, 3);
vecPs2d_w_b = zeros(odo.num, 3);

idxStMk = 6;
for i = 1:mk.numMkId
    vecPt3d_w_m(i,:) = q(idxStMk+i*3-2:idxStMk+i*3).';
end
idxStOdo = idxStMk+3*mk.numMkId;
for i = 1:odo.num
    vecPs2d_w_b(i,:) = q(idxStOdo+3*i-2: idxStOdo+3*i).';
end
vecDt = zeros(numel(time.lp), 1);
for i = 1:numel(time.lp)
    dtTmp = cnstr2period(time.t_mk(i) - time.t_odo(i), 30, -30);
    vecDt(i,1) = dtTmp;
end


%% calculate cost function F
% mark observation part
for i = 1:mk.num
    lp = mk.lp(i);
    lpOdo = find(odo.lp == lp,1);
    x_w_b = vecPs2d_w_b(lpOdo,1);
    y_w_b = vecPs2d_w_b(lpOdo,2);
    theta_w_b = vecPs2d_w_b(lpOdo,3);
    rvec_w_b = [0; 0; theta_w_b];
    tvec_w_b = [x_w_b; y_w_b; 0];
    T3d_w_b = FunVec2Trans3d(rvec_w_b, tvec_w_b);
    
    tvec_c_m = mk.tvec(i,:).';
    mkId = mk.id(i);
    mkIdOrd = find(mk.vecMkId(:,1) == mkId, 1);
    tvec_w_m_model = vecPt3d_w_m(mkIdOrd,:).';
    
    T3d_w_c = T3d_w_b*T3d_b_c;
    tvec_w_m = T3d_w_c(1:3,1:3)*tvec_c_m + T3d_w_c(1:3,4);
    
    dist = norm(tvec_c_m);
    std_z_c2 = max(dist*stdErrRatioMkZ, 0.001);
    std_x_c2 = max(dist*stdErrRatioMkX, 0.001);
    std_y_c2 = max(dist*stdErrRatioMkY, 0.001);
    mat_std_c2 = diag([std_x_c2;std_y_c2;std_z_c2]);
    R3d_c_w = inv(T3d_w_c(1:3,1:3));
    R3d_c_c2 = FunRotPt2ZAxle(tvec_c_m);
    R3d_c2_w = inv(R3d_c_c2)*R3d_c_w;
    mat_std = mat_std_c2*R3d_c2_w;
    
    mats_std_mk{i} = mat_std;
    
    vecCost(3*i-2: 3*i) = inv(mat_std)*(tvec_w_m_model-tvec_w_m);
end

% odometry part
idStOdoOutput = 3*mk.num;
dt_b_c = q(6);
for i = 1:odo.num    
    if i == 1
        x_w_b1 = 0; y_w_b1 = 0; theta_w_b1 = 0;
        ps2d_b1_b2_odo = [0;0;0];
    else
        x_w_b1 = vecPs2d_w_b(i-1,1);
        y_w_b1 = vecPs2d_w_b(i-1,2);
        theta_w_b1 = vecPs2d_w_b(i-1,3);
 
        dt_b1_c1 = (vecDt(i-1) + dt_b_c);
        dt_b2_c2 = (vecDt(i) + dt_b_c);
%         dt_b1_c1 = dt_b_c;
%         dt_b2_c2 = dt_b_c;
        
        ps2d_w_b1_odo = [odo.x(i-1);odo.y(i-1);odo.theta(i-1)];
        ps2d_w_b1t_odo = ps2d_w_b1_odo + ...
            dt_b1_c1*[odo.vx(i-1);odo.vy(i-1);odo.vtheta(i-1)];
        ps2d_w_b2_odo = [odo.x(i);odo.y(i);odo.theta(i)];
        ps2d_w_b2t_odo = ps2d_w_b2_odo + ...
            dt_b2_c2*[odo.vx(i);odo.vy(i);odo.vtheta(i)];        
        
        ps2d_b1_b2_odo = FunRelPos2d(ps2d_w_b1t_odo, ps2d_w_b2t_odo);
    end   
    
    x_w_b2 = vecPs2d_w_b(i,1);
    y_w_b2 = vecPs2d_w_b(i,2);
    theta_w_b2 = vecPs2d_w_b(i,3);
    
    ps2d_w_b1 = [x_w_b1; y_w_b1; theta_w_b1];
    ps2d_w_b2 = [x_w_b2; y_w_b2; theta_w_b2];
    ps2d_b1_b2_mod = FunRelPos2d(ps2d_w_b1, ps2d_w_b2);
    ps2d_b1_b2_mod(3) = FunPrdCnst(ps2d_b1_b2_mod(3), ps2d_b1_b2_odo+pi, ps2d_b1_b2_odo-pi);
    
    std_trans = max(norm(ps2d_b1_b2_odo(1:2))*stdErrRatioOdoLin, 0.001);
    std_rot = max(abs(ps2d_b1_b2_odo(3))*stdErrRatioOdoRot, 0.001*pi/180);    
    
    mat_std = diag([std_trans;std_trans;std_rot]);
    mats_std_odo{i} = mat_std;
    
    vecCost(idStOdoOutput+3*i-2:idStOdoOutput+3*i) = inv(mat_std)*(ps2d_b1_b2_mod-ps2d_b1_b2_odo);
end


%% calculate Jacobian of F
if nargout > 1
    % allocate J
    matJacobian = zeros(3*mk.num+3*odo.num, numel(q));
    
    % compute differentiation of R3d_b_c to rvec_b_c
    [ d_R3d_rvec_b_c_1, d_R3d_rvec_b_c_2, d_R3d_rvec_b_c_3 ] = DiffRotvec2Rotmat( rvec_b_c );
    
    % calculate J of mark observation
    for i = 1:mk.num
        lp = mk.lp(i);
        lpOdo = find(odo.lp == lp,1);
        mkId = mk.id(i);
        mkIdOrd = find(mk.vecMkId(:,1) == mkId, 1);
        
        theta_w_b = vecPs2d_w_b(lpOdo,3);
        R3d_w_b = rodrigues([0;0;theta_w_b]); 
        
        tvec_c_m = mk.tvec(i,:).';
        
        mat_std = mats_std_mk{i};
        
        % Jacoian on camera extrinsic ps2d_b_cg
        % todo ...
        J1 = zeros(3,6);
        J1(1:3, 1) = -R3d_w_b*d_R3d_rvec_b_c_1*tvec_c_m;
        J1(1:3, 2) = -R3d_w_b*d_R3d_rvec_b_c_2*tvec_c_m;
        J1(1:3, 3) = -R3d_w_b*d_R3d_rvec_b_c_3*tvec_c_m;
        J1(1:3, 4:5) = -R3d_w_b(1:3,1:2);        
        matJacobian(3*i-2:3*i, 1:6) = inv(mat_std)*J1;
        
        % Jacobian on marker position pt3d_w_m
        matJacobian(3*i-2:3*i, 3*mkIdOrd+idxStMk-2:3*mkIdOrd+idxStMk) = inv(mat_std)*eye(3);
        
        %Jacobian on robot pose ps2d_w_b
        tvec_b_m = T3d_b_c(1:3,1:3)*tvec_c_m + T3d_b_c(1:3,4);
        R2d_w_b = [cos(theta_w_b) -sin(theta_w_b); sin(theta_w_b) cos(theta_w_b)];
        J3 = zeros(3,3);
        J3(1:2,3) = -R2d_w_b*[-tvec_b_m(2); tvec_b_m(1)];
        J3(1:2,1:2) = -eye(2);
        matJacobian(3*i-2:3*i, idxStOdo+3*lpOdo-2:idxStOdo+3*lpOdo) = inv(mat_std)*J3;
    end
    
    % calculate J of odometry
    row_st = 3*mk.num;
    mat_std = mats_std_odo{1};
    matJacobian(row_st+1:row_st+3, idxStOdo+1:idxStOdo+3) = inv(mat_std);
    for i = 2:odo.num
        ps_w_b1 = vecPs2d_w_b(i-1,:);
        ps_w_b2 = vecPs2d_w_b(i,:);
        [J1, J2] = FunJacobianRelPs2d(ps_w_b1, ps_w_b2);
        mat_std = mats_std_odo{i};
        matJacobian(row_st+3*i-2:row_st+3*i, idxStOdo+3*i-5:idxStOdo+3*i-3) = inv(mat_std)*J1;
        matJacobian(row_st+3*i-2:row_st+3*i, idxStOdo+3*i-2:idxStOdo+3*i) = inv(mat_std)*J2;
        
         % todo: temporal part...
         v_w_b1_odo = [odo.vx(i-1); odo.vy(i-1); odo.vtheta(i-1)];
         v_w_b2_odo = [odo.vx(i); odo.vy(i); odo.vtheta(i)];
         ps_w_b1_odo = [odo.x(i-1); odo.y(i-1); odo.theta(i-1)];
         ps_w_b2_odo = [odo.x(i); odo.y(i); odo.theta(i)];
         
         [J1_odo, J2_odo] = FunJacobianRelPs2d(ps_w_b1_odo, ps_w_b2_odo);
         
         J_t = - J1_odo*v_w_b1_odo - J2_odo*v_w_b2_odo;
         matJacobian(row_st+3*i-2:row_st+3*i, 6) = inv(mat_std)*J_t;         
    end   
end

end

