function [ vec_cost ] = Cost_Spatio( this, vec_mu_x, vec_mu_y )
%COST_SPATIOODO Summary of this function goes here
%   Detailed explanation goes here

qx_c_b = vec_mu_x(1); qy_c_b = vec_mu_x(2); qz_c_b = vec_mu_x(3); qw_c_b = vec_mu_x(4);
x_c_b = vec_mu_x(5); y_c_b = vec_mu_x(6);

x_b2_b1 = vec_mu_y(1); y_b2_b1 = vec_mu_y(2); theta_b2_b1 = vec_mu_y(3);
x_m_c1 = vec_mu_y(4); y_m_c1 = vec_mu_y(5); z_m_c1 = vec_mu_y(6);
x_m_c2 = vec_mu_y(7); y_m_c2 = vec_mu_y(8); z_m_c2 = vec_mu_y(9);

vec_cost = [x_c_b - x_b2_b1 - x_m_c1*(qw_c_b^2 - qx_c_b^2 - qy_c_b^2 + qz_c_b^2) - x_c_b*cos(theta_b2_b1) + y_m_c1*(2*qw_c_b*qx_c_b + 2*qy_c_b*qz_c_b) + z_m_c1*(2*qw_c_b*qy_c_b - 2*qx_c_b*qz_c_b) + y_c_b*sin(theta_b2_b1) + z_m_c2*(2*qx_c_b*qz_c_b*cos(theta_b2_b1) - 2*qw_c_b*qy_c_b*cos(theta_b2_b1) + 2*qw_c_b*qz_c_b*sin(theta_b2_b1) + 2*qx_c_b*qy_c_b*sin(theta_b2_b1)) - x_m_c2*(sin(theta_b2_b1)*(2*qw_c_b*qx_c_b - 2*qy_c_b*qz_c_b) - cos(theta_b2_b1)*(qw_c_b^2 - qx_c_b^2 - qy_c_b^2 + qz_c_b^2)) - y_m_c2*(cos(theta_b2_b1)*(2*qw_c_b*qx_c_b + 2*qy_c_b*qz_c_b) + sin(theta_b2_b1)*(qw_c_b^2 - qx_c_b^2 + qy_c_b^2 - qz_c_b^2));...
    y_c_b - y_b2_b1 - y_m_c1*(qw_c_b^2 - qx_c_b^2 + qy_c_b^2 - qz_c_b^2) - y_c_b*cos(theta_b2_b1) - 2*x_m_c1*(qw_c_b*qx_c_b - qy_c_b*qz_c_b) + z_m_c1*(2*qw_c_b*qz_c_b + 2*qx_c_b*qy_c_b) - x_c_b*sin(theta_b2_b1) - z_m_c2*(2*qw_c_b*qz_c_b*cos(theta_b2_b1) + 2*qx_c_b*qy_c_b*cos(theta_b2_b1) + 2*qw_c_b*qy_c_b*sin(theta_b2_b1) - 2*qx_c_b*qz_c_b*sin(theta_b2_b1)) + x_m_c2*(cos(theta_b2_b1)*(2*qw_c_b*qx_c_b - 2*qy_c_b*qz_c_b) + sin(theta_b2_b1)*(qw_c_b^2 - qx_c_b^2 - qy_c_b^2 + qz_c_b^2)) - y_m_c2*(sin(theta_b2_b1)*(2*qw_c_b*qx_c_b + 2*qy_c_b*qz_c_b) - cos(theta_b2_b1)*(qw_c_b^2 - qx_c_b^2 + qy_c_b^2 - qz_c_b^2));...
    qw_c_b^2*z_m_c1 - qw_c_b^2*z_m_c2 + qx_c_b^2*z_m_c1 - qx_c_b^2*z_m_c2 - qy_c_b^2*z_m_c1 + qy_c_b^2*z_m_c2 - qz_c_b^2*z_m_c1 + qz_c_b^2*z_m_c2 + 2*qw_c_b*qy_c_b*x_m_c1 - 2*qw_c_b*qy_c_b*x_m_c2 + 2*qx_c_b*qz_c_b*x_m_c1 - 2*qx_c_b*qz_c_b*x_m_c2 + 2*qw_c_b*qz_c_b*y_m_c1 - 2*qx_c_b*qy_c_b*y_m_c1 - 2*qw_c_b*qz_c_b*y_m_c2 + 2*qx_c_b*qy_c_b*y_m_c2;...
    qw_c_b^2 + qx_c_b^2 + qy_c_b^2 + qz_c_b^2 - 1];

end

