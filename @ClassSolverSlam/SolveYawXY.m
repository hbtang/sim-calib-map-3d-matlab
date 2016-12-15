function SolveYawXY( this, measure, calib )
% solver step 2: estimate yaw angle and translation from B to C,
% consider linear constraints and one-step optimization.

%% Read data and group constraints
disp('Init: calibrate yaw and xy ...');

odo = measure.odo;
mk = measure.mk;

cnstrRowsLin = {};
cnstrRowsRot = {};

% dl_m/dl < threshLinMotion
% dl_m is the translation of tvec_c_m due to rotation
% dl is the translation from odometry
threshLinMotion = 0.2; 

if isfield(this.setting.solver, 'init_thresh_locallp')
    threshLpLocal = this.setting.solver.init_thresh_locallp;
else
    threshLpLocal = 10;
end

for i = 1:numel(mk.vecMkId)
    mkId_i = mk.vecMkId(i);
    
    vecMkFind = find(mk.id == mkId_i);
    vecMkLpFind = mk.lp(vecMkFind);
    
    for j = 2:numel(vecMkLpFind)
        
        lp1 = vecMkLpFind(j-1);
        lp2 = vecMkLpFind(j);
        
        if abs(lp2-lp1) > threshLpLocal
            continue;
        end
        
        rowOdo1 = find(odo.lp == lp1);
        rowOdo2 = find(odo.lp == lp2);
        
        rowMk1 = vecMkFind(j-1);
        rowMk2 = vecMkFind(j);
        
        cnstrRow.rowOdo1 = rowOdo1;
        cnstrRow.rowOdo2 = rowOdo2;
        cnstrRow.rowMk1 = rowMk1;
        cnstrRow.rowMk2 = rowMk2;
        
        dtheta = cnstr2period(odo.theta(rowOdo1) - odo.theta(rowOdo2), pi, -pi);
        dx = odo.x(rowOdo1) - odo.x(rowOdo2);
        dy = odo.y(rowOdo2) - odo.y(rowOdo2);
        dl = sqrt(dx*dx + dy*dy);
%         rinv = abs(dtheta/dl);
        
        tvec_c1_m = mk.tvec(rowMk1,:);
        dist_c1_m = norm(tvec_c1_m);
        dl_m = abs(dist_c1_m*dtheta);
        
        if abs(dl_m/dl) < threshLinMotion
            cnstrRowsLin{numel(cnstrRowsLin)+1} = cnstrRow;
        else
            cnstrRowsRot{numel(cnstrRowsRot)+1} = cnstrRow;
        end
    end
end

%% Solve yaw with linear motion constraints

ps2d_b_cg = calib.GetPs2dbcg;
T3d_cg_c = calib.T3d_cg_c;
R3d_cg_c = T3d_cg_c(1:3,1:3);
A1 = [];
for i = 1:numel(cnstrRowsLin)
    rowOdo1 = cnstrRowsLin{i}.rowOdo1;
    rowOdo2 = cnstrRowsLin{i}.rowOdo2;
    rowMk1 = cnstrRowsLin{i}.rowMk1;
    rowMk2 = cnstrRowsLin{i}.rowMk2;
    
    x_w_b1 = odo.x(rowOdo1);
    y_w_b1 = odo.y(rowOdo1);
    theta_w_b1 = odo.theta(rowOdo1);
    ps2d_w_b1 = [x_w_b1; y_w_b1; theta_w_b1];
    
    x_w_b2 = odo.x(rowOdo2);
    y_w_b2 = odo.y(rowOdo2);
    theta_w_b2 = odo.theta(rowOdo2);
    ps2d_w_b2 = [x_w_b2; y_w_b2; theta_w_b2];
    
    ps2d_b1_b2 = FunRelPos2d( ps2d_w_b1, ps2d_w_b2 );
    
    p3d_c_m1 = mk.tvec(rowMk1,:).';
    p3d_c_m2 = mk.tvec(rowMk2,:).';
    
    p3d_cg_dm = R3d_cg_c*(p3d_c_m1 - p3d_c_m2);
    
    theta_c = atan2(p3d_cg_dm(2), p3d_cg_dm(1));
    theta_b = atan2(ps2d_b1_b2(2), ps2d_b1_b2(1));
    
    theta_yaw_i = cnstr2period(theta_b - theta_c, pi, -pi);
    
    A1 = [A1; theta_yaw_i];
end

theta_yaw = mean(A1);
ps2d_b_cg(3) = theta_yaw;
calib.SetPs2dbcg(ps2d_b_cg);

%% solve XY_b_cg

A2 = [];
b2 = [];
A3 = [];
b3 = [];

T3d_b_c = calib.T3d_b_c;
R3d_b_c = T3d_b_c(1:3,1:3);

for i = 1:numel(cnstrRowsRot)
    
    rowOdo1 = cnstrRowsRot{i}.rowOdo1;
    rowOdo2 = cnstrRowsRot{i}.rowOdo2;
    rowMk1 = cnstrRowsRot{i}.rowMk1;
    rowMk2 = cnstrRowsRot{i}.rowMk2;
    
    x_w_b1 = odo.x(rowOdo1);
    y_w_b1 = odo.y(rowOdo1);
    theta_w_b1 = odo.theta(rowOdo1);
    ps2d_w_b1 = [x_w_b1; y_w_b1; theta_w_b1];
    
    x_w_b2 = odo.x(rowOdo2);
    y_w_b2 = odo.y(rowOdo2);
    theta_w_b2 = odo.theta(rowOdo2);
    ps2d_w_b2 = [x_w_b2; y_w_b2; theta_w_b2];
    
    ps2d_b1_b2 = FunRelPos2d( ps2d_w_b1, ps2d_w_b2 );
    theta_b1_b2 = ps2d_b1_b2(3);
    p3d_b1_b2 = [ps2d_b1_b2(1); ps2d_b1_b2(2); 0];
    
    p3d_c_m1 = mk.tvec(rowMk1,:).';
    p3d_c_m2 = mk.tvec(rowMk2,:).';
    
    R2d_b1_b2 = [cos(theta_b1_b2) -sin(theta_b1_b2); ...
        sin(theta_b1_b2) cos(theta_b1_b2)];
    R3d_b1_b2 = blkdiag(R2d_b1_b2, 1);
    
    %% A2*x2 = b2, x2 = [x_b_c;y_b_c]
    A2_i = eye(2) - R2d_b1_b2;
    b2_i_bar = R3d_b1_b2*R3d_b_c*p3d_c_m2 - R3d_b_c*p3d_c_m1 + p3d_b1_b2;
    b2_i = b2_i_bar(1:2);    
    A2 = [A2; A2_i];
    b2 = [b2; b2_i];  
    
    %% A3*x3 = b3, x3 = [x_b_c;y_b_c;k_odo_lin]
    A3_i = [eye(2) - R2d_b1_b2, -p3d_b1_b2(1:2)];
    b3_i_bar = R3d_b1_b2*R3d_b_c*p3d_c_m2 - R3d_b_c*p3d_c_m1;
    b3_i = b3_i_bar(1:2);
    A3 = [A3; A3_i];
    b3 = [b3; b3_i];
end

x2 = A2\b2;
x3 = A3\b3;

ps2d_b_cg(1:2) = x3(1:2);
calib.SetPs2dbcg(ps2d_b_cg);

%% return

disp('Init: calibrate yaw and xy done!');

end


