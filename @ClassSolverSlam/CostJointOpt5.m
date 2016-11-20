function [ vecCost, matJacobian ] = CostJointOpt5( this, q, mk, odo, time, calib, setting, flag )
%COST

stdErrRatioMkX = this.errConfig.stdErrRatioMkX;
stdErrRatioMkY = this.errConfig.stdErrRatioMkY;
stdErrRatioMkZ = this.errConfig.stdErrRatioMkZ;
stdErrRatioOdoLin = this.errConfig.stdErrRatioOdoLin;
stdErrRatioOdoRot = this.errConfig.stdErrRatioOdoRot;
MinStdErrOdoLin = this.errConfig.MinStdErrOdoLin;
MinStdErrOdoRot = this.errConfig.MinStdErrOdoRot;

% camera intrinsics
mat_camera = setting.camera.camera_matrix;
% vec_distortion = setting.camera.distortion_coefficients;
vec_distortion = zeros(1,5);

% vecCost: 8*mk.num + 3*odo.num vector of projection error
vecCost = zeros(8*mk.num + 3*odo.num,1);

%% parse q: rvec_b_c(1:3), tvec_b_c(1:2), vecPt3d_w_m, vecPs2d_w_b
numParam = flag.bCalibExt*5 + flag.bCalibTmp + flag.bCalibOdo*2;
idxStMk = numParam;
q_param = q(1:idxStMk);
count = 1;
if flag.bCalibExt
    rvec_b_c = q_param(count:count+2);
    tvec_b_c = [q_param(count+3:count+4);0];
    vecrow_ext = count:count+4;
    count = count+5;
else
    rvec_b_c = calib.rvec_b_c;
    tvec_b_c = calib.tvec_b_c;
end
if flag.bCalibTmp
    dt_b_c = q_param(count);
    vecrow_tmp = count;
    count = count + 1;
else
    dt_b_c = calib.dt;
end
if flag.bCalibOdo
    k_odo_lin = q_param(count);
    k_odo_rot = q_param(count+1);
    vecrow_odo = count:count+1;
else
    k_odo_lin = calib.k_odo_lin;
    k_odo_rot = calib.k_odo_rot;
end
[ T3d_b_c, R3d_b_c ] = FunVec2Trans3d( rvec_b_c, tvec_b_c );

vec_rvec_w_m = zeros(mk.numMkId, 3);
vec_tvec_w_m = zeros(mk.numMkId, 3);
vec_ps2d_w_b = zeros(odo.num, 3);
for i = 1:mk.numMkId
    vec_rvec_w_m(i,:) = q(idxStMk+i*6-5:idxStMk+i*6-3).';
    vec_tvec_w_m(i,:) = q(idxStMk+i*6-2:idxStMk+i*6).';
end
idxStOdo = idxStMk+6*mk.numMkId;
for i = 1:odo.num
    vec_ps2d_w_b(i,:) = q(idxStOdo+3*i-2: idxStOdo+3*i).';
end
vecDt = zeros(numel(time.lp), 1);
for i = 1:numel(time.lp)
    dtTmp = cnstr2period(time.t_mk(i) - time.t_odo(i), 30, -30);
    vecDt(i,1) = dtTmp;
end
tvec_m_pt1 = setting.aruco.tvec_m_pt1.';
tvec_m_pt2 = setting.aruco.tvec_m_pt2.';
tvec_m_pt3 = setting.aruco.tvec_m_pt3.';
tvec_m_pt4 = setting.aruco.tvec_m_pt4.';

%% Compute Cost Function
% mark observation part: image offset between model and undistorted
% cordinates
for i = 1:mk.num
    lp = mk.lp(i);
    row_odo = find(odo.lp == lp,1);
    x_w_b = vec_ps2d_w_b(row_odo,1);
    y_w_b = vec_ps2d_w_b(row_odo,2);
    theta_w_b = vec_ps2d_w_b(row_odo,3);
    rvec_w_b = [0; 0; theta_w_b];
    tvec_w_b = [x_w_b; y_w_b; 0];
    T3d_w_b = FunVec2Trans3d(rvec_w_b, tvec_w_b);
    
    mkId = mk.id(i);
    mkIdOrd = find(mk.vecMkId(:,1) == mkId, 1);
    
    rvec_w_m = vec_rvec_w_m(mkIdOrd,:).';
    tvec_w_m = vec_tvec_w_m(mkIdOrd,:).';
    T3d_w_m = FunVec2Trans3d(rvec_w_m, tvec_w_m);
    
    T3d_c_m = inv(T3d_w_b*T3d_b_c)*T3d_w_m;
    [ rvec_c_m, tvec_c_m ] = FunTrans2Vec3d( T3d_c_m );
    
    img_c_pt1_measure = mk.pt1(i,:).';
    img_c_pt2_measure = mk.pt2(i,:).';
    img_c_pt3_measure = mk.pt3(i,:).';
    img_c_pt4_measure = mk.pt4(i,:).';
    
    img_c_pt1 = ProjXYZ2UV( rvec_c_m, tvec_c_m, tvec_m_pt1, mat_camera, vec_distortion );
    img_c_pt2 = ProjXYZ2UV( rvec_c_m, tvec_c_m, tvec_m_pt2, mat_camera, vec_distortion );
    img_c_pt3 = ProjXYZ2UV( rvec_c_m, tvec_c_m, tvec_m_pt3, mat_camera, vec_distortion );
    img_c_pt4 = ProjXYZ2UV( rvec_c_m, tvec_c_m, tvec_m_pt4, mat_camera, vec_distortion );
    
    errimg_c_pt1 = img_c_pt1 - img_c_pt1_measure;
    errimg_c_pt2 = img_c_pt2 - img_c_pt2_measure;
    errimg_c_pt3 = img_c_pt3 - img_c_pt3_measure;
    errimg_c_pt4 = img_c_pt4 - img_c_pt4_measure;
    
    vec_err_temp = [errimg_c_pt1; errimg_c_pt2; errimg_c_pt3; errimg_c_pt4];
    
    mat_std = eye(8);
    mats_std_mk{i} = mat_std;
    vecCost(8*i-7: 8*i) = inv(mat_std)*(vec_err_temp);
end

% odometry part
idStOdoOutput = 8*mk.num;

for i = 1:odo.num
    if i == 1
        x_w_b1 = 0; y_w_b1 = 0; theta_w_b1 = 0;
        ps2d_b1_b2_odo = [0;0;0];
    else
        x_w_b1 = vec_ps2d_w_b(i-1,1);
        y_w_b1 = vec_ps2d_w_b(i-1,2);
        theta_w_b1 = vec_ps2d_w_b(i-1,3);
        
        dt_b1_c1 = (vecDt(i-1) + dt_b_c);
        dt_b2_c2 = (vecDt(i) + dt_b_c);
        
        ps2d_w_b1_odo = [odo.x(i-1);odo.y(i-1);odo.theta(i-1)];
        ps2d_w_b1t_odo = ps2d_w_b1_odo + ...
            dt_b1_c1*[odo.vx(i-1);odo.vy(i-1);odo.vtheta(i-1)];
        ps2d_w_b2_odo = [odo.x(i);odo.y(i);odo.theta(i)];
        ps2d_w_b2t_odo = ps2d_w_b2_odo + ...
            dt_b2_c2*[odo.vx(i);odo.vy(i);odo.vtheta(i)];
        
        ps2d_b1_b2_odo = FunRelPos2d(ps2d_w_b1t_odo, ps2d_w_b2t_odo);
    end
    ps2d_b1_b2_odo(1:2) = ps2d_b1_b2_odo(1:2)*k_odo_lin;
    ps2d_b1_b2_odo(3) = ps2d_b1_b2_odo(3)*k_odo_rot;
    
    x_w_b2 = vec_ps2d_w_b(i,1);
    y_w_b2 = vec_ps2d_w_b(i,2);
    theta_w_b2 = vec_ps2d_w_b(i,3);
    
    ps2d_w_b1 = [x_w_b1; y_w_b1; theta_w_b1];
    ps2d_w_b2 = [x_w_b2; y_w_b2; theta_w_b2];
    ps2d_b1_b2_mod = FunRelPos2d(ps2d_w_b1, ps2d_w_b2);
    ps2d_b1_b2_mod(3) = FunPrdCnst(ps2d_b1_b2_mod(3), ps2d_b1_b2_odo+pi, ps2d_b1_b2_odo-pi);
    
    std_trans = max(norm(ps2d_b1_b2_odo(1:2))*stdErrRatioOdoLin, MinStdErrOdoLin);
    std_rot = max(abs(ps2d_b1_b2_odo(3))*stdErrRatioOdoRot, MinStdErrOdoRot);
    
    mat_std = diag([std_trans;std_trans;std_rot]);
    mats_std_odo{i} = mat_std;
    
    vecCost(idStOdoOutput+3*i-2:idStOdoOutput+3*i) = inv(mat_std)*(ps2d_b1_b2_mod-ps2d_b1_b2_odo);
end


%% Compute Jacobian
if nargout > 1
    % allocate J
    matJacobian = zeros(8*mk.num+3*odo.num, numel(q));
    
    % calculate J of mark observation
    for i = 1:mk.num
        lp = mk.lp(i);
        row_odo = find(odo.lp == lp,1);
        mkId = mk.id(i);
        mkIdOrd = find(mk.vecMkId(:,1) == mkId, 1);
        
        rvec_w_m = vec_rvec_w_m(mkIdOrd,:).';
        tvec_w_m = vec_tvec_w_m(mkIdOrd,:).';
        
        tvec_w_b = [vec_ps2d_w_b(row_odo,1); vec_ps2d_w_b(row_odo,2); 0];
        rvec_w_b = [0;0;vec_ps2d_w_b(row_odo,3)];
        
        [~,J_pt1_bc, J_pt1_wb, J_pt1_wm] = Jacobian_Opt5Mk_Lie( ...
            rvec_b_c, tvec_b_c, rvec_w_b, tvec_w_b, rvec_w_m, tvec_w_m,...
            tvec_m_pt1, mat_camera, vec_distortion);
        [~,J_pt2_bc, J_pt2_wb, J_pt2_wm] = Jacobian_Opt5Mk_Lie( ...
            rvec_b_c, tvec_b_c, rvec_w_b, tvec_w_b, rvec_w_m, tvec_w_m,...
            tvec_m_pt2, mat_camera, vec_distortion);
        [~,J_pt3_bc, J_pt3_wb, J_pt3_wm] = Jacobian_Opt5Mk_Lie( ...
            rvec_b_c, tvec_b_c, rvec_w_b, tvec_w_b, rvec_w_m, tvec_w_m,...
            tvec_m_pt3, mat_camera, vec_distortion);
        [~,J_pt4_bc, J_pt4_wb, J_pt4_wm] = Jacobian_Opt5Mk_Lie( ...
            rvec_b_c, tvec_b_c, rvec_w_b, tvec_w_b, rvec_w_m, tvec_w_m,...
            tvec_m_pt4, mat_camera, vec_distortion);
        
        J_pt_bc = [J_pt1_bc; J_pt2_bc; J_pt3_bc; J_pt4_bc];
        J_pt_wb = [J_pt1_wb; J_pt2_wb; J_pt3_wb; J_pt4_wb];
        J_pt_wm = [J_pt1_wm; J_pt2_wm; J_pt3_wm; J_pt4_wm];
        
        vecrow_jac = i*8-7:i*8;
        veccol_jac_wb = idxStOdo+3*row_odo-2:idxStOdo+3*row_odo;
        veccol_jac_wm = idxStMk+6*mkIdOrd-5:idxStMk+6*mkIdOrd;
        
        matJacobian(vecrow_jac, veccol_jac_wb) = J_pt_wb;
        matJacobian(vecrow_jac, veccol_jac_wm) = J_pt_wm;
        
        if flag.bCalibExt
            veccol_jac_bc = vecrow_ext;
            matJacobian(vecrow_jac, veccol_jac_bc) = J_pt_bc;
        end
    end
    
    % calculate J of odometry
    row_st = 8*mk.num;
    mat_std = mats_std_odo{1};
    matJacobian(row_st+1:row_st+3, idxStOdo+1:idxStOdo+3) = inv(mat_std);
    for i = 2:odo.num
        ps_w_b1 = vec_ps2d_w_b(i-1,:);
        ps_w_b2 = vec_ps2d_w_b(i,:);
        [J1, J2] = FunJacobianRelPs2d(ps_w_b1, ps_w_b2);
        mat_std = mats_std_odo{i};
        matJacobian(row_st+3*i-2:row_st+3*i, idxStOdo+3*i-5:idxStOdo+3*i-3) = inv(mat_std)*J1;
        matJacobian(row_st+3*i-2:row_st+3*i, idxStOdo+3*i-2:idxStOdo+3*i) = inv(mat_std)*J2;
        
        % temporal part ...
        v_w_b1_odo = [odo.vx(i-1); odo.vy(i-1); odo.vtheta(i-1)];
        v_w_b2_odo = [odo.vx(i); odo.vy(i); odo.vtheta(i)];
        ps_w_b1_odo = [odo.x(i-1); odo.y(i-1); odo.theta(i-1)];
        ps_w_b2_odo = [odo.x(i); odo.y(i); odo.theta(i)];
        ps_b1_b2_odo = FunRelPos2d(ps_w_b1_odo, ps_w_b2_odo);
        if flag.bCalibTmp
            [J1_odo, J2_odo] = FunJacobianRelPs2d(ps_w_b1_odo, ps_w_b2_odo);
            J_t = - J1_odo*v_w_b1_odo - J2_odo*v_w_b2_odo;
            matJacobian(row_st+3*i-2:row_st+3*i, vecrow_tmp) = inv(mat_std)*J_t;
        end
        
        % odometric part ...
        J_odo = zeros(3,2);
        J_odo(1:2,1) = -ps_b1_b2_odo(1:2);
        J_odo(3,2) = -ps_b1_b2_odo(3);
        if flag.bCalibOdo
            matJacobian(row_st+3*i-2:row_st+3*i, vecrow_odo) = inv(mat_std)*J_odo;
        end
    end
end

end

