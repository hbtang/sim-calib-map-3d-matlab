function SolveLocalLoopOpt(this, measure, calib)
%SOLVELOCALLOOPOPT alternative solution of step 2, local optimization with
% loop closing info, but no slam

disp(['Start calibration with local and loop closing info... ']);

q = calib.GetPs2dbcg();

%% solve nonlinear least square, mapping and calibration
options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
    'Display', 'iter-detailed', 'Jacobian', 'on', 'MaxIter',50, ...
    'TolX', 1e-6);

[q,resnorm,residual,exitflag,output,lambda,jacobian] = ...
    lsqnonlin(@(x)this.CostLocalLoopOpt(x, measure, calib), q, [], [], options);

% debug start
% figure;
% plot(residual);
% debug end

calib.SetPs2dbcg(q);

disp(['Calibration with local and loop closing info is done!']);
[U,S,V] = svd(jacobian);
disp('svd result of jacobian matrix:');
disp(['S eigen value: ', num2str(S(1,1)), ' ',...
    num2str(S(2,2)), ' ',num2str(S(3,3)), ' ']);
disp('V:');
disp(V);
disp(' ');


end

