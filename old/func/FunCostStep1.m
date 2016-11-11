function [ cost ] = FunCostStep1( q, mark )
%FUNCOSTSTEP1 cost function for points to plane
% sum of dist^2
vDist = [];
for i = (1:mark.num)
    id_ord_i = find(mark.mkHash == mark.id(i));  
    vGrnd_i = [q(1:3); q(id_ord_i+3)];
    dist_i = FunDistPt2Plane(vGrnd_i,mark.tvec(i,:).');
    vDist(end+1,1) = dist_i;
end
cost = sqrt(vDist.'*vDist/mark.num);


end

