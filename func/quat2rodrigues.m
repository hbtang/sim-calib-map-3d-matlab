function [ r ] = quat2rodrigues( q )
% q = [qx; qy; qz; qw] = [n*sin(theta/2); cos(theta/2)]

if norm(q) == 0
    error('Error, quat2rodrgues');
end

q = q/norm(q);
theta = acos(q(4))*2;

if theta == 0
    r = [0;0;0];
else
    r = q(1:3)/sin(theta/2)*theta;
end

end

