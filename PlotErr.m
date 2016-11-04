function PlotErr( cell_mat, vec_idx, text, bAxisEqual )
% Plot error with multiple input data

if nargin == 3
    bAxisEqual = true;
end

figure;

set(gcf, 'Position', [800,1,640,480]);
hold on; grid on; box on;
if bAxisEqual
    axis equal; 
end

title(text.strTitle, 'FontWeight','bold');
xlabel(text.strXLabel);
ylabel(text.strYLabel);

cmdColor = {'b' 'g' 'r'};

ratioStd = 2;

for i = 1:numel(cell_mat)
    mat_err = cell_mat{i}(:, vec_idx);
    mat_cov = cov(mat_err);
    vec_mean = mean(mat_err);
    vecEllipse = Ellipse_CovMean( vec_mean, mat_cov, ratioStd);
    
    plot(mat_err(:,1), mat_err(:,2), '.', 'Color', cmdColor{i});
    plot(vecEllipse(:,1), vecEllipse(:,2), '-', 'Color', cmdColor{i}, 'LineWidth', 2); 
end

end

