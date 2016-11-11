function [ calib ] = ProcEstStep2( mark, odo, calib )
%PROCESTSTEP2 2nd step in calibration
% calibrate the remaining 3 dof camera extrinsic
% based on ground vector in camera frame, from 1st step

%% initial guess
vec_pt3d_o_m = zeros(mark.numSeg,3);
ps2d_b_cg = [0;0;-pi/2];

% obtain T_b_c in 3d
rvec_b_cg = [0;0;ps2d_b_cg(3)];
tvec_b_cg = [ps2d_b_cg(1:2);0];
T_b_cg = FunVec2Trans3d(rvec_b_cg, tvec_b_cg);

% obtain rvet_cg_c: c is camera frame, cg is ground projected camera frame
rvec_cg_c = FunGrnd2Rot(calib.vGrnd); tvec_cg_c = [0;0;0];
T_cg_c = FunVec2Trans3d(rvec_cg_c, tvec_cg_c);

% obtain T_b_c in 3d
T_b_c = T_b_cg*T_cg_c;

calib.T_cg_c = T_cg_c;

% set vec_pt3d_o_m, 3d point coordinates of all marker
% w.r.t. odometry frame
% same id but in different mark observation segement considered as
% different mark here

for i = 1:mark.numSeg    
    stampMkSt = mark.seg(i,1);
    stamp = mark.stamp(stampMkSt);
    x_o_b = odo.x(stamp);
    y_o_b = odo.y(stamp);
    theta_o_b = odo.theta(stamp);
    rvec_o_b = [0; 0; theta_o_b];
    tvec_o_b = [x_o_b; y_o_b; 0];
    T_o_b = FunVec2Trans3d(rvec_o_b, tvec_o_b);
    
    pt3d_c_m = mark.tvec(stampMkSt,:).';
    pt3d_o_m = T_o_b*T_b_c*[pt3d_c_m; 1];
    pt3d_o_m = pt3d_o_m(1:3);
    
    vec_pt3d_o_m(i, :) = pt3d_o_m.';
end

% define variable vector vec_q
vec_q = [ps2d_b_cg];
for i = 1:mark.numSeg
    vec_q = [vec_q; vec_pt3d_o_m(i,:).'];
end

%% solve nonlinear least square

% debuging code ...
% vec_q_delta = vec_q;
% col = 3;
% vec_q_delta(col) = vec_q_delta(col) + 0.001;
% F_ref = FunCostStep2(vec_q, mark, odo, T_cg_c);
% F_delta = FunCostStep2(vec_q_delta, mark, odo, T_cg_c);
% J_finit_col = (F_delta-F_ref)/0.001;
% [F,J] = FunCostStep2(vec_q, mark, odo, T_cg_c);
% plot(J_finit_col - J(:,col))
% debuging code end ...

options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
    'Display', 'iter', 'Jacobian', 'on', 'DerivativeCheck', 'off');
[vec_q] = lsqnonlin(@(x)FunCostStep2(x, mark, odo, calib), vec_q, [], [], options);

%% save and display results
ps2d_b_cg = vec_q(1:3);
rvec_b_cg = [0;0;ps2d_b_cg(3)];
tvec_b_cg = [ps2d_b_cg(1:2);0];
[T_b_cg] = FunVec2Trans3d(rvec_b_cg, tvec_b_cg);
T_b_c = T_b_cg*T_cg_c;

% save calibration result in calib
calib.T_cg_c = T_cg_c;
calib.ps2d_b_cg = ps2d_b_cg;
rvec_b_cg = [0;0;ps2d_b_cg(3)];
tvec_b_cg = [ps2d_b_cg(1:2);0];
calib.T_b_cg = FunVec2Trans3d(rvec_b_cg, tvec_b_cg);
calib.T_b_c = T_b_c;
[ calib.rvec_cg_c, calib.tvec_cg_c ] = FunTrans2Vec3d( T_cg_c );
[ calib.rvec_b_cg, calib.tvec_b_cg ] = FunTrans2Vec3d( T_b_cg );
[ calib.rvec_b_c, calib.tvec_b_c ] = FunTrans2Vec3d( T_b_c );

% show results
% figure; grid on; hold on; axis equal;
% view(3);
% view(0, 90);
% plot3(vec_q(4:3:end), vec_q(5:3:end), vec_q(6:3:end), ...
%     '.', 'Color','k','MarkerSize',10);
% plot(odo.x, odo.y, '.-', 'Color','b','MarkerSize',5);

end

