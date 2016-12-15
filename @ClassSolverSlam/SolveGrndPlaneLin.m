function SolveGrndPlaneLin( this, measure, calib )

% solver step 1: estimate ground plane in camera frame to obtain pvec_g_c
% and dist_g_c, consider linear constraints and one-step optimization.

%% obtain initial guess in linear model
disp('Init: calibrate ground plane ...');

mk = measure.mk;
A = zeros(mk.num, 3+mk.numMkId);
for i = 1:mk.num
    A(i,1:3) = mk.tvec(i,:);
    mkIdOrd = find(mk.vecMkId == mk.id(i));
    A(i, 3+mkIdOrd) = 1;
end
[~,~,V] = svd(A);

% q: result vector, defined as [pvec_g_c; w_m_c], w_m_c(i) is the fourth
% term of "mark ground plane", which is perpendicular to the ground but go
% through the related mark.

[rowV, colV] = size(V);
V2 = zeros(rowV, colV);
idx_best = 0;
norm_residual_best = inf;

for i = 1:colV
  vi = V(:,i);
  vi2 = vi./norm(vi(1:3));
  V2(:,i) = vi2;
  vec_residual = A*vi2;
  norm_residual = norm(vec_residual);
  if norm_residual < norm_residual_best
      idx_best = i;
      norm_residual_best = norm_residual;
  end
end
q = V2(:,idx_best);

%% set y_c point to ground plane negative norm direction
if q(2) > 0
    q = -q;
end

%% refresh calib
calib.pvec_g_c = q(1:3);
calib.dist_g_c = 0;
calib.RefreshByGrnd;

%% output result
% this.DrawResGrndPlane(measure, calib, q);
disp('Init: calibrate ground plane done!');
disp(' ');

end

