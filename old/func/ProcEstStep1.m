function [ vGrnd, vMkH ] = ProcEstStep1( mark )
%PROCGRNDPLANEMKHIGHT: 
% estimate ground plane vector w.r.t. camera
% estimate the hight of all marks

%% linear model, get intial guess fast
% q = [x_g, y_g, z_g, w_mk1, w_mk2, ..., w_mkm]
% A = [x1, y1, z1, 0, 0, ... 1, 0, ...]

A = zeros(mark.num, 3+mark.numId);
for i = 1:mark.num
    A(i,1:3) = mark.tvec(i,:);
    id_ord_i = find(mark.mkHash == mark.id(i));
    A(i, 3+id_ord_i) = 1;
end
[U,S,V] = svd(A);
q = V(:,end);
q = q./norm(q(1:3));
cost = FunCostPts2Plane( q, mark );
disp(['ground estimation initialized, with cost value: ', num2str(cost)]);

%% nonlinear model, refine the estimation
% q = fmincon(@(x)FunCostPts2Plane(x, mark), q,[],[],[],[],[],[],@ConUnitVec3);
% q = fminunc(@(x)FunCostPts2Plane(x, mark), q, options);
options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'Display', 'iter');
q = lsqnonlin(@(x)FunCostPts2PlaneLsq(x, mark), q, [], [], options);
q = q./norm(q(1:3));
cost = FunCostPts2Plane( q, mark );
disp(['ground estimation done, with cost value: ', num2str(cost)]);

%% calculate error vector: distance from points to related plane 
vDist = [];
vDist = FunCostPts2PlaneLsq(q, mark);

vGrnd = [q(1:3);0];
vMkH = q(4:end);

%% draw results
% figure;
% plot(vDist);

% figure;
% hold on;
% axis equal;
% view(90,0);
% % view(3);
% plot3(mark.tvec(:,1), mark.tvec(:,2), mark.tvec(:,3), '.');
% [X,Z] = meshgrid(-1000:50:1000, 500:50:2000);
% Y = -(X.*q(1) + Z.*q(3))./q(2);
% mesh(X,Y,Z)

end

