function PrintRmsErr( err_Mk, err_Odo, err_MkNorm, err_OdoNorm )
% to print rmse

%% Show rmse
rmse_Mk = rms(err_Mk);
rmse_Odo = rms(err_Odo);
rmse_MkNorm = rms(err_MkNorm);
rmse_OdoNorm = rms(err_OdoNorm);
disp(['rmse_Mk: ', num2str(rmse_Mk(1)), ' ', ...
    num2str(rmse_Mk(2)), ' ', num2str(rmse_Mk(3)), ' ']);
disp(['rmse_Odo: ', num2str(rmse_Odo(1)), ' ', ...
    num2str(rmse_Odo(2)), ' ', num2str(rmse_Odo(3)), ' ']);
disp(['rmse_MkNorm: ', num2str(rmse_MkNorm(1)), ' ', ...
    num2str(rmse_MkNorm(2)), ' ', num2str(rmse_MkNorm(3)), ' ']);
disp(['rmse_OdoNorm: ', num2str(rmse_OdoNorm(1)), ' ', ...
    num2str(rmse_OdoNorm(2)), ' ', num2str(rmse_OdoNorm(3)), ' ']);

end

