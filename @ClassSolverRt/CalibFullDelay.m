function CalibFullDelay( this )
%CALIBFULL online calibration algorithm of full camera extrinsic parameters
% to be done ...

%% load calibration info from last iteration
% mu_x = [q_c_b(1:4); pt3_c_b(1:2); dt] = [qx; qy; qz; qw; x; y; dt]
mu_x = this.x_full_dt;
sigma_x = this.sigma_x_full_dt;

%% load new mark info
mkNew = this.mkNew;
mkRec = this.mkRec;

%% return if no mark observed
if isempty(mkNew)
    return;
end

%% return if current mark is not observed
vecMkId = intersect(mkNew.id, mkRec.id);
if isempty(vecMkId)
    return;
end

%% main loop
for i = 1:numel(vecMkId)
    % find mark info location
    mkId_i = vecMkId(i);
    rowRec = find(mkRec.id == mkId_i,1);
    rowNew = find(mkNew.id == mkId_i,1);
            
    % generate measurements Y: y = [ps2_b2_b1; pt3_m_c1; pt3_m_c2]
    [mu_y, sigma_y, odoinfo] = CreateYFullDelay(this, rowNew, rowRec);
    
    % generate virtual measurements Z
    [mu_z, sigma_z, jacobian_z] = CreateZFullDelay(this, mu_x, sigma_x, mu_y, sigma_y, odoinfo);
        
    % correct XY based on Z
    [mu_xy_corr, sigma_xy_corr] = CorrectXYFullDelay(this, ...
        mu_x, sigma_x, mu_y, sigma_y, mu_z, sigma_z, jacobian_z, odoinfo);
    
    % marginalization on Y
    mu_x_corr = mu_xy_corr(1:7);
    sigma_x_corr = sigma_xy_corr(1:7,1:7);
    
    % low bound of sigma
    [U,S,V] = svd(sigma_x_corr(1:4,1:4));
    S = max(S,eye(4)*1e-6);
    sigma_x_corr(1:4,1:4) = U*S*V.';
    
    [U,S,V] = svd(sigma_x_corr(5:6,5:6));
    S = max(S,eye(2)*1e-2);
    sigma_x_corr(5:6,5:6) = U*S*V.';
    
    % update
    mu_x = mu_x_corr;
    sigma_x = sigma_x_corr;
    
end

%% update estimation result
% if (mu_x(1) < 0)
%     mu_x(1:4) = -mu_x(1:4);
% end
this.q_c_b = mu_x(1:4);
this.pt3_c_b(1:2) = mu_x(5:6);
this.dt = mu_x(7);
this.x_full = mu_x(1:6);
this.sigma_x_full = sigma_x(1:6,1:6);
this.sigma_q_c_b = sigma_x(1:4,1:4);
this.sigma_pt3_c_b = sigma_x(5:6,5:6);
this.x_full_dt = mu_x(1:7);
this.sigma_x_full_dt = sigma_x;

end


