function PropagateCovOdo( this, lp_now, measure )
%PROPAGATECOVODO Summary of this function goes here
%   Detailed explanation goes here

row_now = find(measure.odo.lp == lp_now, 1);
if row_now == 1
    measure.odo.cov{row_now,1} = zeros(3,3);
    return;
end

row_last = row_now - 1;

stdErrRatioOdoLin = this.errConfig.stdErrRatioOdoLin;
stdErrRatioOdoRot = this.errConfig.stdErrRatioOdoRot;
% MinStdErrOdoLin = errConfig.MinStdErrOdoLin;
% MinStdErrOdoRot = errConfig.MinStdErrOdoRot;

odo = measure.odo;

dx = odo.x(row_now) - odo.x(row_last);
dy = odo.y(row_now) - odo.y(row_last);
dtheta = odo.theta(row_now) - odo.theta(row_last);
dtheta = cnstr2period(dtheta, pi, -pi);
dl = norm([dx;dy]);

std_dxy = dl*stdErrRatioOdoLin;
std_dtheta = abs(dtheta)*stdErrRatioOdoRot;

sigma_d = diag([std_dxy^2;std_dxy^2;std_dtheta^2]);
sigma_last = measure.odo.cov{row_last};
sigma_now = sigma_last + sigma_d;

measure.odo.cov{row_now,1} = sigma_now;

end

