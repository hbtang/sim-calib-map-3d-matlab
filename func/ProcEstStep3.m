function [ calib, map ] = ProcEstStep3( mark, odo, calib )
%PROCESTSTEP2 2nd step in calibration
% calibrate the remaining 3 dof camera extrinsic
% based on ground vector in camera frame, from 1st step

%% initial guess
map.id = mark.mkHash(:,1);
map.vecPt3d_w_m = zeros(mark.numId, 3);
map.vecPs2d_w_b = zeros(odo.num, 3);
T_b_c = calib.T_b_c;

% init map.vecPt3d_w_m, 3d point coordinates of marks in world frame
for i = 1:mark.numId
    idMkNow = mark.mkHash(i,1);    
    stampMkNow = find(mark.id == idMkNow, 1);
    stampNow = mark.stamp(stampMkNow);
    
    x_w_b = odo.x(stampNow);
    y_w_b = odo.y(stampNow);
    theta_o_b = odo.theta(stampNow);
    rvec_o_b = [0; 0; theta_o_b];
    tvec_o_b = [x_w_b; y_w_b; 0];
    T_w_b = FunVec2Trans3d(rvec_o_b, tvec_o_b);
    
    pt3d_c_m = mark.tvec(stampMkNow,:).';
    T_w_c = T_w_b*T_b_c;
    pt3d_w_m = T_w_c(1:3,1:3)*pt3d_c_m + T_w_c(1:3,4);
    
    map.vecPt3d_w_m(i,:) = pt3d_w_m.';
end

% init map.vecPs2d_w_b, 2d pose of robot base in world frame
for i = 1:odo.num
    map.vecPs2d_w_b(i,:) = [odo.x(i); odo.y(i); odo.theta(i)].';
end

% define variable vector vec_q: [ps2d_b_cg;map.vecPt3d_w_m;map.vecPs2d_w_b]
vec_q = zeros(3+3*mark.numId+3*odo.num,1);
vec_q(1:3) = calib.ps2d_b_cg;
for i = 1:mark.numId
    vec_q(i*3+1:i*3+3) = map.vecPt3d_w_m(i,:).';
end
for i = 1:odo.num
    idxSt = 3+3*mark.numId;
    vec_q(idxSt+3*i-2: idxSt+3*i) = map.vecPs2d_w_b(i,:).';
end

%% debuging code ...
% vec_q_delta = vec_q;
% col = 400;
% vec_q_delta(col) = vec_q_delta(col) + 0.001;
% F_ref = FunCostStep3(vec_q, mark, odo, calib);
% F_delta = FunCostStep3(vec_q_delta, mark, odo, calib);
% J_finit_col = (F_delta-F_ref)/0.001;
% [F,J] = FunCostStep3(vec_q, mark, odo, calib);
% figure;
% plot(J_finit_col - J(:,col));
% figure;
% plot(J_finit_col);
% figure;
% plot(J(:,col));
% disp('debuging');
% debuging code end ...

%% solve nonlinear least square, mapping and calibration
options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
    'Display', 'iter', 'Jacobian', 'on', 'DerivativeCheck', 'off', 'MaxIter',10);
[vec_q, resnorm, residual ] = lsqnonlin(@(x)FunCostStep3(x, mark, odo, calib), vec_q, [], [], options);
% disp(resnorm);
% plot(residual);

%% save results
% parse vec_q
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

% save in map
map.vecPt3d_w_m = vecPt3d_w_m;
map.vecPs2d_w_b = vecPs2d_w_b;

% save calibration result in calib
calib.ps2d_b_cg = ps2d_b_cg;
rvec_b_cg = [0;0;ps2d_b_cg(3)];
tvec_b_cg = [ps2d_b_cg(1:2);0];
[T_b_cg] = FunVec2Trans3d(rvec_b_cg, tvec_b_cg);
T_b_c = T_b_cg*calib.T_cg_c;
calib.T_b_cg = FunVec2Trans3d(rvec_b_cg, tvec_b_cg);
calib.T_b_c = T_b_c;
[ calib.rvec_cg_c, calib.tvec_cg_c ] = FunTrans2Vec3d( calib.T_cg_c );
[ calib.rvec_b_cg, calib.tvec_b_cg ] = FunTrans2Vec3d( calib.T_b_cg );
[ calib.rvec_b_c, calib.tvec_b_c ] = FunTrans2Vec3d( calib.T_b_c );

%% show results
% figure; grid on; hold on; axis equal;
% plot3(vecPt3d_w_m(:,1), vecPt3d_w_m(:,2), vecPt3d_w_m(:,3), ...
%     '.', 'Color','k','MarkerSize',5);
% plot(vecPs2d_w_b(:,1), vecPs2d_w_b(:,2),...
%      '.-', 'Color','b','MarkerSize',5);
% plot(odo.x, odo.y,...
%      '.-', 'Color','g','MarkerSize',5);
% for i = 1:mark.num
%     idMk = mark.id(i);
%     idMkOrd = find(mark.mkHash(:,1) == idMk);
%     stamp = mark.stamp(i);
%     pt3d_w_m = vecPt3d_w_m(idMkOrd,:).';
%     pt3d_w_b = vecPs2d_w_b(stamp,:).';
%     pt3d_w_b(3) = 0;    
%     plot3([pt3d_w_m(1);pt3d_w_b(1)], [pt3d_w_m(2);pt3d_w_b(2)], [pt3d_w_m(3);pt3d_w_b(3)], ...
%        '-', 'Color','r');
% end

end

