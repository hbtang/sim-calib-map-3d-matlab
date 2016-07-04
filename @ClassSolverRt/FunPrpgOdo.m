function  [odoNew_out] = FunPrpgOdo(this, odoLast, odoNew)
%FUNPRPGODO propagate covariance matrix of odometry
%   refresh odoNew.sigma

errRatioTr = this.errConfig.errRatioOdoTr;
errRatioRot = this.errConfig.errRatioOdoRot;

dx = odoNew.x - odoLast.x;
dy = odoNew.y - odoLast.y;
dtheta = odoNew.theta - odoLast.theta;
dtheta = cnstr2period(dtheta, pi, -pi);
dl = norm([dx;dy]);

std_dxy = dl*errRatioTr;
std_dtheta = abs(dtheta)*errRatioRot;

sigma_d = diag([std_dxy^2;std_dxy^2;std_dtheta^2]);

odoNew_out = odoNew;
odoNew_out.sigma = odoLast.sigma + sigma_d;

end

