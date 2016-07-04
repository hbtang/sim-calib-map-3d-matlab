function SolveGrndPlane( this, measure, calib )
%SOLVESTEP1: solve step 1
% solver step 1: estimate ground plane in camera frame to obtain pvec_g_c
% and dist_g_c

%% obtain initial guess in linear model
disp(['Start calibrating ground plane in camera frame...']);

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

q = V(:,end);
q = q./norm(q(1:3));
cost = this.CostGrndPlane( q, mk );
disp(['Ground estimation initialized.']);

%% refine estimation in nonlinear model
options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'Display', 'iter', 'ScaleProblem', 'Jacobian');
q = lsqnonlin(@(x)this.CostGrndPlane( x, mk ), q, [], [], options);
q = q./norm(q(1:3));
% set y_c point to ground plane negative norm direction
if q(2) > 0
    q = -q;
end

% cost = this.CostStep1( q, mk );
disp(['Ground plane calibration done!']);
disp(' ');

%% refresh calib
calib.pvec_g_c = q(1:3);
calib.dist_g_c = 0;
calib.RefreshByGrnd;

%% draw result
% this.DrawResGrndPlane(measure, calib);

end

