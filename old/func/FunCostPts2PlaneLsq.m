function [ vDist ] = FunCostPts2PlaneLsq( q, mark )
%FUNCOSTPTS2PLANELSQ Summary of this function goes here
% return vector of distance to according plane
% for nonlinear least square optimization: lsqnonlin()
vDist = [];
for i = (1:mark.num)
    id_ord_i = find(mark.mkHash == mark.id(i));  
    vGrnd_i = [q(1:3); q(id_ord_i+3)];
    dist_i = FunDistPt2Plane(vGrnd_i,mark.tvec(i,:).');
    vDist(end+1,1) = dist_i;
end
end

