function [ vecCost, matJacobian ] = CostLocalLoopOpt(this, q, measure, calib)
%COSTLOCALOPT alternative solution of step 2: local optimization with loop
% closing info, but no slam.

%% parse vector q and init
vecCost = [];
matJacobian = [];
stdErrRatioOdoLin = this.errConfig.stdErrRatioOdoLin;
odo = measure.odo;
mk = measure.mk;
odo.l = zeros(odo.num,1);

ps2d_b_cg = q;
T3d_b_cg = FunPs2d2T3d(ps2d_b_cg);

T3d_cg_c = calib.T3d_cg_c;
T3d_b_c = T3d_b_cg*T3d_cg_c;


for i = 2:odo.num
    dx = odo.x(i) - odo.x(i-1);
    dy = odo.y(i) - odo.y(i-1);
    dl = norm([dx;dy]);
    odo.l(i) = odo.l(i-1) + dl;
end

for i = 1:numel(mk.vecMkId)
    mkId = mk.vecMkId(i);
    vecMkRaw = find(mk.id == mkId);
    
    for j = 2:numel(vecMkRaw)
        idMkRaw1 = vecMkRaw(j-1);
        idMkRaw2 = vecMkRaw(j);
        
        lp1 = mk.lp(idMkRaw1);
        lp2 = mk.lp(idMkRaw2);
        
        idOdoRaw1 = find(odo.lp == lp1, 1);
        idOdoRaw2 = find(odo.lp == lp2, 1);
        
        ps2d_o_b1 = [odo.x(idOdoRaw1); odo.y(idOdoRaw1); odo.theta(idOdoRaw1)];
        ps2d_o_b2 = [odo.x(idOdoRaw2); odo.y(idOdoRaw2); odo.theta(idOdoRaw2)];
        
        T3d_o_b1 = FunPs2d2T3d(ps2d_o_b1);
        T3d_o_b2 = FunPs2d2T3d(ps2d_o_b2);
        
        rvec_c1_m = mk.rvec(idMkRaw1, :).';
        tvec_c1_m = mk.tvec(idMkRaw1, :).';
        T3d_c1_m = FunVec2Trans3d(rvec_c1_m, tvec_c1_m);
        
        rvec_c2_m = mk.rvec(idMkRaw2, :).';
        tvec_c2_m = mk.tvec(idMkRaw2, :).';
        T3d_c2_m = FunVec2Trans3d(rvec_c2_m, tvec_c2_m);
        
        T3d_o_c1 = T3d_o_b1*T3d_b_c;
        T3d_o_c2 = T3d_o_b2*T3d_b_c;
        
        tvec_o_m_1 = T3d_o_c1(1:3,1:3)*tvec_c1_m + T3d_o_c1(1:3,4);
        tvec_o_m_2 = T3d_o_c2(1:3,1:3)*tvec_c2_m + T3d_o_c2(1:3,4);
        
        err_tvec_o_m = tvec_o_m_1(1:2) - tvec_o_m_2(1:2);
        
        dl = odo.l(idOdoRaw2) - odo.l(idOdoRaw1);
        
        stdErr = max(stdErrRatioOdoLin*dl, 0.001);
        
        err_tvec_o_m = diag([1/stdErr;1/stdErr])*err_tvec_o_m;
        
        vecCost = [vecCost; err_tvec_o_m];
        
        % generate jacobian matrix
        x_o_b1 = ps2d_o_b1(1); y_o_b1 = ps2d_o_b1(2); theta_o_b1 = ps2d_o_b1(3);
        x_o_b2 = ps2d_o_b2(1); y_o_b2 = ps2d_o_b2(2); theta_o_b2 = ps2d_o_b2(3);
        x_b_cg = ps2d_b_cg(1); y_b_cg = ps2d_b_cg(2); theta_b_cg = ps2d_b_cg(3);
        
        tvec_cg1_m = T3d_cg_c(1:3,1:3)*tvec_c1_m+T3d_cg_c(1:3,4);
        tvec_cg2_m = T3d_cg_c(1:3,1:3)*tvec_c2_m+T3d_cg_c(1:3,4);
        
        x_cg1_m = tvec_cg1_m(1); y_cg1_m = tvec_cg1_m(2);
        x_cg2_m = tvec_cg2_m(1); y_cg2_m = tvec_cg2_m(2);        
        
        matJacTmp = [cos(theta_o_b1) - cos(theta_o_b2), sin(theta_o_b2) - sin(theta_o_b1), y_cg2_m*cos(theta_o_b2 + theta_b_cg) - y_cg1_m*cos(theta_o_b1 + theta_b_cg) - x_cg1_m*sin(theta_o_b1 + theta_b_cg) + x_cg2_m*sin(theta_o_b2 + theta_b_cg);...
            sin(theta_o_b1) - sin(theta_o_b2), cos(theta_o_b1) - cos(theta_o_b2), x_cg1_m*cos(theta_o_b1 + theta_b_cg) - x_cg2_m*cos(theta_o_b2 + theta_b_cg) - y_cg1_m*sin(theta_o_b1 + theta_b_cg) + y_cg2_m*sin(theta_o_b2 + theta_b_cg)];
        matJacTmp = diag([1/stdErr;1/stdErr])*matJacTmp;
        matJacobian = [matJacobian; matJacTmp];
    end
end
end

