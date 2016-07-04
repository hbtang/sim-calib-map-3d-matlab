%% camera extrinsic param
syms qx_c_b qy_c_b qz_c_b qw_c_b
q_c_b = [qx_c_b; qy_c_b; qz_c_b; qw_c_b];
R3_c_b = quat2rot(q_c_b);
syms x_c_b y_c_b
t3_c_b = [x_c_b; y_c_b; 0];
T3_c_b = [R3_c_b t3_c_b; 0 0 0 1];

%% odometry
syms x_b2_b1 y_b2_b1 theta_b2_b1
q_b2_b1 = [cos(theta_b2_b1/2); 0; 0; -sin(theta_b2_b1/2)];
R3_b2_b1 = quat2rot(q_b2_b1);
t3_b2_b1 = [x_b2_b1; y_b2_b1; 0];
T3_b2_b1 = [R3_b2_b1 t3_b2_b1; 0 0 0 1];

%% mark observation
syms x_m_c1 y_m_c1 z_m_c1
pt3_m_c1 = [x_m_c1; y_m_c1; z_m_c1];

syms x_m_c2 y_m_c2 z_m_c2
pt3_m_c2 = [x_m_c2; y_m_c2; z_m_c2];

%% constraint
pt3_b1_m_1 = T3_c_b*[pt3_m_c1;1];
pt3_b1_m_1(4) = [];

pt3_b1_m_2 = T3_b2_b1*T3_c_b*[pt3_m_c2;1];
pt3_b1_m_2(4) = [];

err = pt3_b1_m_1 - pt3_b1_m_2;
err = simplify(err);
err(end+1) = q_c_b.'*q_c_b - 1;

%% testing

% val_pt3_m_c1 = [-1291.0902 0.053542 2347.6486].';
% val_pt3_m_c2 = [1835.8901 -0.07613 3376.7435].';
% val_ps2_b1_o = [2496.8845 4044.6043 1.4326].';
% val_ps2_b2_o = [8443.6101 3292.8273 -1.5331].';
% 
% % val_pt3_m_c1 = [0 0 2000].';
% % val_pt3_m_c2 = [2000 0 -1000].';
% % val_ps2_b1_o = [0 0 0].';
% % val_ps2_b2_o = [1000 0 pi/2].';
% 
% val_ps2_b2_b1 = FunRelPos2d(val_ps2_b1_o, val_ps2_b2_o);
% 
% val_q_c_b = [0 0 -0.7071 0.7071].';
% val_t3_c_b = [0;0;0];
% 
% vec_sym = [qx_c_b qy_c_b qz_c_b qw_c_b ...
%     x_c_b y_c_b ...
%     x_b2_b1 y_b2_b1 theta_b2_b1 ...
%     x_m_c1 y_m_c1 z_m_c1 ...
%     x_m_c2 y_m_c2 z_m_c2
%     ];
% vec_val = [val_q_c_b; val_t3_c_b(1:2); val_ps2_b2_b1; val_pt3_m_c1; val_pt3_m_c2].';
% 
% val_err = subs(err, vec_sym, vec_val);

%% Hessian and Jacobian

vec_sym = [qx_c_b qy_c_b qz_c_b qw_c_b ...
    x_c_b y_c_b ...
    x_b2_b1 y_b2_b1 theta_b2_b1 ...
    x_m_c1 y_m_c1 z_m_c1 ...
    x_m_c2 y_m_c2 z_m_c2
    ];

J1 = simplify(jacobian(err(1), vec_sym));
H1 = simplify(jacobian(J1, vec_sym));

J2 = simplify(jacobian(err(2), vec_sym));
H2 = simplify(jacobian(J2, vec_sym));

J3 = simplify(jacobian(err(3), vec_sym));
H3 = simplify(jacobian(J3, vec_sym));

J4 = simplify(jacobian(err(4), vec_sym));
H4 = simplify(jacobian(J4, vec_sym));

spy_sym(H1)




