function [ q ] = rodrigues2quat( r )
% q = [qx; qy; qz; qw] = [n*sin(theta/2); cos(theta/2)]

theta = norm(r);

if theta == 0
    q = [0;0;0;1];
else
    vec_norm = r/theta;
    q = [sin(theta/2)*vec_norm; cos(theta/2)];
end

end

