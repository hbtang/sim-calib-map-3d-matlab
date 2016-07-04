function [ cost ] = CostGrndPlane( this, q, mk )
%COSTSTEP1 Summary of this function goes here
%   Detailed explanation goes here

vecDist = zeros(mk.num, 1);
for i = (1:mk.num)
    mkId = find(mk.vecMkId == mk.id(i));  
    pvec_g_c = q(1:3);
    
    % homogeneous vector of "ground mark plane"
    phvec_gm_c = [pvec_g_c; q(mkId+3)];
    
    dist = FunDistPt2Plane(phvec_gm_c, mk.tvec(i,:).');
    vecDist(i,1) = dist;
end
cost = vecDist;

end

