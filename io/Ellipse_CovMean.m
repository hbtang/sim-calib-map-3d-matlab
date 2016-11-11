function [ vec ] = Ellipse_CovMean( mean, cov, ratio )
% Create ellipse trajectory to plot
vec = [];
mean_x = mean(1);
mean_y = mean(2);

lambda = eig(cov);
if lambda(1)<0 || lambda(2)<0
    error('Error!');
end    

[U,S,V] = svd(cov);
a = ratio*sqrt(S(1,1));
b = ratio*sqrt(S(2,2));
va = V(:,1);
vb = V(:,2);

phi = atan2(va(2), va(1));
vec_theta = 0:1*pi/180:2*pi;

for i = 1:numel(vec_theta)
    theta = vec_theta(i);
    x = mean_x + a*cos(theta)*cos(phi) - b*sin(theta)*sin(phi);
    y = mean_y + a*cos(theta)*sin(phi) + b*sin(theta)*cos(phi);
    
    vec = [vec; x y];    
end

end

