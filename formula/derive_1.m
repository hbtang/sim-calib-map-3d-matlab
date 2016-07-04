% derive constraint that between two observation of the same mark, 2nd
% tylor expansion

syms q0 q1 q2 q3 x_b_c y_b_c z_b_c
syms x_b1_b2 y_b1_b2 theta_b1_b2
syms x_c1_m y_c1_m z_c1_m
syms x_c2_m y_c2_m z_c2_m

R_b_c = [q0*q0+q1*q1-q2*q2-q3*q3 2*q1*q2+2*q0*q3 2*q1*q3-2*q0*q2;...
    2*q1*q2-2*q0*q3 q0*q0-q1*q1+q2*q2-q3*q3 2*q2*q3+2*q0*q1;...
    2*q1*q3+2*q0*q2 2*q2*q3-2*q0*q1 q0*q0-q1*q1-q2*q2+q3*q3];

p_b_c = [x_b_c; y_b_c; z_b_c];
T_b_c = [R_b_c p_b_c; 0 0 0 1];

R_b1_b2 = [cos(theta_b1_b2) -sin(theta_b1_b2) 0;...
    sin(theta_b1_b2) cos(theta_b1_b2) 0;...
    0 0 1];

p_b1_b2 = [x_b1_b2; y_b1_b2; 0];
T_b1_b2 = [R_b1_b2 p_b1_b2; 0 0 0 1];

p1_b1_m = T_b_c*[x_c1_m; y_c1_m; z_c1_m; 1];
p2_b1_m = T_b1_b2*T_b_c*[x_c2_m; y_c2_m; z_c2_m; 1];
dp_b1_m = p1_b1_m-p2_b1_m;

% simple(dp_b1_m)

x = [q0 q1 q2 q3 x_b_c y_b_c z_b_c].';
y = [x_b1_b2 y_b1_b2 theta_b1_b2 x_c1_m y_c1_m z_c1_m x_c2_m y_c2_m z_c2_m].';
xy = [x;y];

J1 = jacobian(dp_b1_m(1), xy);
H1 = jacobian(J1,xy);
H1 = simplify(H1)
Z1 = zeros(16,16);
for i = 1:16
    for j = 1:16        
        if H1(i,j) ~= 0
            Z1(i,j) = 1;
        end       
    end
end
figure;
spy(Z1);

J2 = jacobian(dp_b1_m(2), xy);
H2 = jacobian(J2,xy);
H2 = simplify(H2)
Z2 = zeros(16,16);
for i = 1:16
    for j = 1:16        
        if H2(i,j) ~= 0
            Z2(i,j) = 1;
        end       
    end
end
figure;
spy(Z2);

J3 = jacobian(dp_b1_m(3), xy);
H3 = jacobian(J3,xy);
H3 = simplify(H3)
Z3 = zeros(16,16);
for i = 1:16
    for j = 1:16        
        if H3(i,j) ~= 0
            Z3(i,j) = 1;
        end       
    end
end
figure;
spy(Z3);

    
    
    
    











