function SolveLocalOpt(this, measure, calib)
%SOLVELOCALOPT solve step 2:
% generate initial guess of full calibration problem,
% according to local observation, no initial guess needed in this step.

disp(['Start calibration with local infomation... ']);

q = calib.GetPs2dbcg();

%% solve nonlinear least square, mapping and calibration
options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
    'Display', 'iter-detailed', 'Jacobian', 'on', 'MaxIter',50, ...
    'TolX', 1e-6);

[q,resnorm,residual,exitflag,output,lambda,jacobian] = ...
    lsqnonlin(@(x)this.CostLocalOpt(x, measure, calib), q, [], [], options);

calib.SetPs2dbcg(q);

disp(['Calibration with local infomation done!']);
[U,S,V] = svd(jacobian);
disp('svd result of jacobian matrix:');
disp(['S eigen value: ', num2str(S(1,1)), ' ',...
    num2str(S(2,2)), ' ',num2str(S(3,3)), ' ']);
disp('V:');
disp(V);
disp(' ');


end

