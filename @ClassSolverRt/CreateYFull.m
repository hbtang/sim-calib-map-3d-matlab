function [ mu_y, sigma_y ] = CreateYFull( this, rowNew, rowRec )
%CREATEYFULL create measurements Y in full calibration
% y = []

stdRatioErrXY = this.errConfig.errRatioMkLtr;
stdRatioErrZ = this.errConfig.errRatioMkDpt;

pt3_m_c1 = this.mkRec.tvec(rowRec,:).';
pt3_m_c2 = this.mkNew.tvec(rowNew,:).';

lp1 = this.mkRec.lp(rowRec);

rowOdo1 = find(this.odoRec.lp == lp1);

ps2_o_b1 = [this.odoRec.x(rowOdo1);this.odoRec.y(rowOdo1);this.odoRec.theta(rowOdo1)];
ps2_o_b2 = [this.odoNew.x; this.odoNew.y; this.odoNew.theta];

ps2_b2_b1 = FunRelPos2d( ps2_o_b1, ps2_o_b2 );

sigma_ps2_b1_o = this.odoRec.sigma{rowOdo1};
sigma_ps2_b2_o = this.odoNew.sigma;

sigma_ps2_b2_b1 = sigma_ps2_b2_o-sigma_ps2_b1_o;

depth_m_c1 = abs(pt3_m_c1(3));
std_x_m_c1 = stdRatioErrXY*depth_m_c1;
std_y_m_c1 = stdRatioErrXY*depth_m_c1;
std_z_m_c1 = stdRatioErrZ*depth_m_c1;
sigma_pt3_m_c1 = diag([std_x_m_c1^2; std_y_m_c1^2; std_z_m_c1^2]);

depth_m_c2 = abs(pt3_m_c2(3));
std_x_m_c2 = stdRatioErrXY*depth_m_c2;
std_y_m_c2 = stdRatioErrXY*depth_m_c2;
std_z_m_c2 = stdRatioErrZ*depth_m_c2;
sigma_pt3_m_c2 = diag([std_x_m_c2^2; std_y_m_c2^2; std_z_m_c2^2]);

mu_y = [ps2_b2_b1; pt3_m_c1; pt3_m_c2];

sigma_y = max(blkdiag(sigma_ps2_b2_b1, sigma_pt3_m_c1, sigma_pt3_m_c2), eye(9)*1e-6);


end

