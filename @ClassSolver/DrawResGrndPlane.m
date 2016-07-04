function DrawResGrndPlane( this, measure, calib )
%DRAWSTEP1 draw estimation result of step 1
if isempty(this.hdFigSolver)
    this.hdFigSolver = figure;
end
figure(this.hdFigSolver);
axis equal; grid on; hold on;
view(3);
set(this.hdFigSolver, 'Name', 'Estimate Result of Step 1', 'NumberTitle', 'off');

mk = measure.mk;
pvec_g_c = calib.pvec_g_c;
plot3(mk.tvec(:,1), mk.tvec(:,2), mk.tvec(:,3), '.', 'Color', 'k');
[X,Z] = meshgrid(-1000:50:1000, 500:50:2000);
Y = -(X.*pvec_g_c(1) + Z.*pvec_g_c(3))./pvec_g_c(2);
mesh(X,Y,Z);

end

