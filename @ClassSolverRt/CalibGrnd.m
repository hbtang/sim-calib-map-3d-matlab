function CalibGrnd(this)
%CALIBGRNDPL online calibration algorithm of ground plane in camera frame
%% init
% load current plane info
mu_x = this.pl_g_c;
sigma_x = this.sigma_pl_g_c;

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
    
    % load mark observation
    pt3d_m_c1 = this.mkRec.tvec(rowRec,:).';
    pt3d_m_c2 = this.mkNew.tvec(rowNew,:).';
    
    % generate measurement Y = pt3d_m_c2 - pt3d_m_c1    
    [mu_y, sigma_y] = CreateYGrnd(this, ...
        pt3d_m_c2, pt3d_m_c1);
    
    % generate virtual measurements Z
    [mu_z, sigma_z, jacobian_z] = CreateZGrnd(this, ...
        mu_x, sigma_x, mu_y, sigma_y);
        
    % correction based on z
    [mu_xy_corr, sigma_xy_corr] = CorrectXYGrnd(this, mu_x, sigma_x, ...
        mu_y, sigma_y, mu_z, sigma_z, jacobian_z);
    
    % marginalization on dpt_mc
    mu_x_corr = mu_xy_corr(1:3);
    sigma_x_corr = sigma_xy_corr(1:3,1:3);
    
    % low bound of sigma
    [U,S,V] = svd(sigma_x_corr);
    S = max(S,eye(3)*1e-6);
    sigma_x_corr = U*S*V.';
    
    % update
    mu_x = mu_x_corr;
    sigma_x = sigma_x_corr;
    
end

%% update estimation result
this.pl_g_c = mu_x;
this.sigma_pl_g_c = sigma_x;

end

