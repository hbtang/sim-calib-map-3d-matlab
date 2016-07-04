syms theta

R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
I = eye(2);

Q = I-R;

det(Q)