function [ error ] = GetCalibError( result, truth )
%GETCALIBERROR compute calibration error

error.tvec_b_c = result.tvec_b_c - truth.tvec_b_c;
error.rvec_b_c = drvec(result.rvec_b_c, truth.rvec_b_c);
error.dt_b_c = result.dt_b_c - truth.dt_b_c;
error.k_odo_lin = result.k_odo_lin - truth.k_odo_lin;
error.k_odo_rot = result.k_odo_rot - truth.k_odo_rot;

error.norm_tvec = norm(error.tvec_b_c);
error.norm_rvec = norm(error.rvec_b_c);

end

