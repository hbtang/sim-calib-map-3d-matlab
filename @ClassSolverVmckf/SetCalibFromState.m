function SetCalibFromState(this, calib)

qvec_b_c = this.vec_mu_x(1:4);
rvec_b_c = quat2rodrgues(qvec_b_c);
tvec_b_c = [this.vec_mu_x(5:6); 0];
dt = this.vec_mu_x(7);

calib.rvec_b_c = rvec_b_c;
calib.tvec_b_c = tvec_b_c;
calib.dt = dt;
calib.RefreshByVecbc

end

