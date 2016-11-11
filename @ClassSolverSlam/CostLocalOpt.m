function [ vecCost, matJacobian ] = CostLocalOpt(this, q, measure, calib)
%COSTLOCALOPT cost function of local optimization

%% parse vector q and init
vecCost = [];
matJacobian = [];

odo = measure.odo;
mk = measure.mk;

ps2d_b_cg = q;
T3d_b_cg = FunPs2d2T3d(ps2d_b_cg);

T3d_cg_c = calib.T3d_cg_c;
T3d_b_c = T3d_b_cg*T3d_cg_c;

for i = 2:odo.num
    lp1 = odo.lp(i-1);
    lp2 = odo.lp(i);
    
    ps2d_o_b1 = [odo.x(i-1);odo.y(i-1);odo.theta(i-1)];
    ps2d_o_b2 = [odo.x(i);odo.y(i);odo.theta(i)];
    T3d_o_b1 = FunPs2d2T3d(ps2d_o_b1);
    T3d_o_b2 = FunPs2d2T3d(ps2d_o_b2);
    
    vecMkRow1 = find(mk.lp == lp1);
    vecMkRow2 = find(mk.lp == lp2);
    
    vecMkId1 = mk.id(vecMkRow1);
    vecMkId2 = mk.id(vecMkRow2);
    vecMkSame = intersect(vecMkId1 ,vecMkId2);
    
    if isempty(vecMkSame)
        continue;
    end
    
    for j = 1:numel(vecMkSame)
        mkId = vecMkSame(j);
        
        mkRow1 = vecMkRow1(find(vecMkId1 == mkId),1);
        mkRow2 = vecMkRow2(find(vecMkId2 == mkId),1);
        
        rvec_c1_m = mk.rvec(mkRow1, :).';
        rvec_c2_m = mk.rvec(mkRow2, :).';
        tvec_c1_m = mk.tvec(mkRow1, :).';
        tvec_c2_m = mk.tvec(mkRow2, :).';
        T3d_c1_m = FunVec2Trans3d(rvec_c1_m, tvec_c1_m);
        T3d_c2_m = FunVec2Trans3d(rvec_c2_m, tvec_c2_m);
        
        T3d_o_m_1 = T3d_o_b1*T3d_b_c*T3d_c1_m;
        T3d_o_m_2 = T3d_o_b2*T3d_b_c*T3d_c2_m;
        
        cost = T3d_o_m_1(1:2,4) - T3d_o_m_2(1:2,4);
        vecCost = [vecCost; cost];
        
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
        matJacobian = [matJacobian; matJacTmp];
    end
end

end

