%% proof: q is observable with only pure rotation

syms v1 q0 q1 q2 q3

Q0 = [q0 q3 -q2; -q3 q0 q1; q2 -q1 q0];
Q1 = [q1 q2 q3; q2 -q1 q0; q3 -q0 -q1];
Q2 = [-q2 q1 -q0; q1 q2 q3; q0 q3 -q2];
Q3 = [-q3 q0 q1; -q0 -q3 q2; q1 q2 q3];

syms a0 a1 a2 a3

Q = a0*Q0+a1*Q1+a2*Q2+a3*Q3;
R = quat2rot([q0 q1 q2 q3]);

R_INV = R.';
R_INV_2 = R_INV(:, 1:2);

P = simplify(Q*R_INV_2);
% eig(Q)

v1 = [q0 q1 q2 q3];
v2 = [q3 -q2 q1 -q0];
v3 = [q2 q3 -q0 -q1];
v4 = [q1 -q0 -q3 q2];

V = [v1; v2; v3; v4];
det(V)

W = [q0 q1 q2 q3; q3 -q2 q1 -q0; -q2 -q3 q0 q1; q1 -q0 -q3 q2];