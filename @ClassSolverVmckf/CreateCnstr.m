function cell_cnstr = CreateCnstr( this, batch, measure )
%CREATECNSTR Summary of this function goes here
%   Detailed explanation goes here

stdErrRatioMkX = this.errConfig.stdErrRatioMkX;
stdErrRatioMkY = this.errConfig.stdErrRatioMkY;
stdErrRatioMkZ = this.errConfig.stdErrRatioMkZ;
stdErrRatioOdoLin = this.errConfig.stdErrRatioOdoLin;
stdErrRatioOdoRot = this.errConfig.stdErrRatioOdoRot;
MinStdErrOdoLin = this.errConfig.MinStdErrOdoLin;
MinStdErrOdoRot = this.errConfig.MinStdErrOdoRot;

cell_cnstr = {};
odo = measure.odo;
mk = measure.mk;

id = batch.id;
vec_lp = batch.vec_lp;

rows_mk_obs = find(mk.id == id);

if numel(vec_lp) < 2
    return;
end

% for i = 1:numel(vec_lp)
for i = 2:numel(vec_lp)
    
    % indices
    if i == 1
        lp1 = vec_lp(1);
        lp2 = vec_lp(end);
    else
        lp1 = vec_lp(i-1);
        lp2 = vec_lp(i);
    end
    row_odo_1 = find(odo.lp == lp1, 1);
    row_odo_2 = find(odo.lp == lp2, 1);
    row_mk_1 = intersect(find(mk.lp == lp1), rows_mk_obs);
    row_mk_2 = intersect(find(mk.lp == lp2), rows_mk_obs);
    
    % odometry
    ps_w_b1 = [odo.x(row_odo_1); odo.y(row_odo_1); odo.theta(row_odo_1)];
    ps_w_b2 = [odo.x(row_odo_2); odo.y(row_odo_2); odo.theta(row_odo_2)];
    vps_w_b1 = [odo.vx(row_odo_1); odo.vy(row_odo_1); odo.vtheta(row_odo_1)];
    vps_w_b2 = [odo.vx(row_odo_2); odo.vy(row_odo_2); odo.vtheta(row_odo_2)];
    ps_b1_b2 = FunRelPos2d(ps_w_b1, ps_w_b2);
%     std_tran = max(norm(ps_b1_b2(1:2))*stdErrRatioOdoLin, MinStdErrOdoLin);
%     std_rot = max(abs(ps_b1_b2(3))*stdErrRatioOdoRot, MinStdErrOdoRot);
%     Sigma_ps_b1_b2 = blkdiag(std_tran*std_tran*eye(2), std_rot*std_rot);
%     if lp2-lp1 > 1000
%         Sigma_ps_b1_b2 = blkdiag(eye(2)*400, (5*pi/180)^2);
%     end
    Sigma_ps_b1_b2 = odo.cov{row_odo_2} - odo.cov{row_odo_1};
    
    % mark
    tvec_c1_m = mk.tvec(row_mk_1, :).';
    dist1 = norm(tvec_c1_m);
    depth1 = tvec_c1_m(3);
    std_z_c1 = max(depth1*stdErrRatioMkZ, 1);
    std_x_c1 = max(depth1*stdErrRatioMkX, 1);
    std_y_c1 = max(depth1*stdErrRatioMkY, 1);
    mat_std_c1 = diag([std_x_c1;std_y_c1;std_z_c1]);
    Sigma_tvec_c1_m = mat_std_c1*mat_std_c1;
    
    tvec_c2_m = mk.tvec(row_mk_2, :).';
    dist2 = norm(tvec_c2_m);
    depth2 = tvec_c2_m(3);
    std_z_c2 = max(depth2*stdErrRatioMkZ, 1);
    std_x_c2 = max(depth2*stdErrRatioMkX, 1);
    std_y_c2 = max(depth2*stdErrRatioMkY, 1);
    mat_std_c2 = diag([std_x_c2;std_y_c2;std_z_c2]);
    Sigma_tvec_c2_m = mat_std_c2*mat_std_c2;
    
    % return
    cnstr_temp = struct(...
        'lp1', lp1, 'lp2', lp2, 'id', id,...
        'ps_w_b1', ps_w_b1, 'ps_w_b2', ps_w_b2,...
        'vps_w_b1', vps_w_b1, 'vps_w_b2', vps_w_b2,...
        'ps_b1_b2', ps_b1_b2, 'Sigma_ps_b1_b2', Sigma_ps_b1_b2,...
        'tvec_c1_m', tvec_c1_m, 'tvec_c2_m', tvec_c2_m,...
        'Sigma_tvec_c1_m', Sigma_tvec_c1_m, 'Sigma_tvec_c2_m', Sigma_tvec_c2_m...
        );
    
    cell_cnstr{end+1, 1} = cnstr_temp;
end


end

