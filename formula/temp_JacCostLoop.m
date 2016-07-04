syms x_o_b1 y_o_b1 theta_o_b1;
syms x_o_b2 y_o_b2 theta_o_b2;
syms x_b_cg y_b_cg theta_b_cg;
syms x_cg1_m y_cg1_m;
syms x_cg2_m y_cg2_m;

T2d_b_cg = [cos(theta_b_cg) -sin(theta_b_cg) x_b_cg;...
    sin(theta_b_cg) cos(theta_b_cg) y_b_cg;...
    0 0 1];

T2d_o_b1 = [cos(theta_o_b1) -sin(theta_o_b1) x_o_b1;...
    sin(theta_o_b1) cos(theta_o_b1) y_o_b1;...
    0 0 1];

T2d_o_b2 = [cos(theta_o_b2) -sin(theta_o_b2) x_o_b2;...
    sin(theta_o_b2) cos(theta_o_b2) y_o_b2;...
    0 0 1];

e = T2d_o_b1*T2d_b_cg*[x_cg1_m;y_cg1_m;1] - T2d_o_b2*T2d_b_cg*[x_cg2_m;y_cg2_m;1];
e = simplify(e);

Jx = simplify(diff(e,x_b_cg));
Jy = simplify(diff(e,y_b_cg));
Jtheta = simplify(diff(e,theta_b_cg));

J = [Jx,Jy,Jtheta]

