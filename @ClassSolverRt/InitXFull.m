function InitXFull(this, q_c_b, sigma_q_c_b, pt3_c_b, sigma_pt3_c_b)
%INITXFULL Summary of this function goes here
% set initial guess for full calibration
this.q_c_b = q_c_b;
this.sigma_q_c_b = sigma_q_c_b;
this.pt3_c_b = pt3_c_b;
this.sigma_pt3_c_b = sigma_pt3_c_b;

this.x_full = [this.q_c_b; this.pt3_c_b(1:2)];
this.sigma_x_full = blkdiag(this.sigma_q_c_b, this.sigma_pt3_c_b(1:2,1:2));
end

