function Draw( this )
%DRAW draw or refresh current status

%% calculate pts to draw
% base
ps2d_w_b = this.ps2d_w_b;
T3d_w_b = FunPs2d2T3d(ps2d_w_b);
pt3d_w_b0 = T3d_w_b(1:3,4);
pt3d_w_b1 = T3d_w_b(1:3,1:3)*[1000;0;0] + T3d_w_b(1:3,4);
pt3d_w_b2 = T3d_w_b(1:3,1:3)*[0;1000;0] + T3d_w_b(1:3,4);
pt3d_w_b3 = T3d_w_b(1:3,1:3)*[0;0;1000] + T3d_w_b(1:3,4);

% camera
T3d_b_c = this.calib.T3d_b_c;
T3d_w_c = T3d_w_b*T3d_b_c;

pts3d_c_range = zeros(8,3);
distRangeMin = this.setting.range.depth_min;
distRangeMax = this.setting.range.depth_max;
angleOfViewH = this.setting.range.angleofview_h;
angleOfViewV = this.setting.range.angleofview_v;

x_close = tan(angleOfViewH/2)*distRangeMin;
y_close = tan(angleOfViewV/2)*distRangeMin;
x_far = tan(angleOfViewH/2)*distRangeMax;
y_far = tan(angleOfViewV/2)*distRangeMax;

pts3d_c_range(1:4,3) = distRangeMin;
pts3d_c_range(5:8,3) = distRangeMax;
pts3d_c_range(1,1:2) = [x_close, y_close];
pts3d_c_range(2,1:2) = [-x_close, y_close];
pts3d_c_range(3,1:2) = [-x_close, -y_close];
pts3d_c_range(4,1:2) = [x_close, -y_close];
pts3d_c_range(5,1:2) = [x_far, y_far];
pts3d_c_range(6,1:2) = [-x_far, y_far];
pts3d_c_range(7,1:2) = [-x_far, -y_far];
pts3d_c_range(8,1:2) = [x_far, -y_far];

pt3d_w_range = zeros(8,3);
for i = 1:8
    pt3d_w_range(i,:) = (T3d_w_c(1:3,1:3)*pts3d_c_range(i,:).' + T3d_w_c(1:3,4)).';
end

X_range = zeros(4,6); Y_range = zeros(4,6); Z_range = zeros(4,6);
X_range(:,1) = pt3d_w_range([1 2 3 4],1);
Y_range(:,1) = pt3d_w_range([1 2 3 4],2);
Z_range(:,1) = pt3d_w_range([1 2 3 4],3);
X_range(:,2) = pt3d_w_range([5 6 7 8],1);
Y_range(:,2) = pt3d_w_range([5 6 7 8],2);
Z_range(:,2) = pt3d_w_range([5 6 7 8],3);
X_range(:,3) = pt3d_w_range([1 4 8 5],1);
Y_range(:,3) = pt3d_w_range([1 4 8 5],2);
Z_range(:,3) = pt3d_w_range([1 4 8 5],3);
X_range(:,4) = pt3d_w_range([1 2 6 5],1);
Y_range(:,4) = pt3d_w_range([1 2 6 5],2);
Z_range(:,4) = pt3d_w_range([1 2 6 5],3);
X_range(:,5) = pt3d_w_range([2 3 7 6],1);
Y_range(:,5) = pt3d_w_range([2 3 7 6],2);
Z_range(:,5) = pt3d_w_range([2 3 7 6],3);
X_range(:,6) = pt3d_w_range([3 7 8 4],1);
Y_range(:,6) = pt3d_w_range([3 7 8 4],2);
Z_range(:,6) = pt3d_w_range([3 7 8 4],3);

%% on first draw
if isempty(this.hds.hdFigSim)
    % init
    this.hds.hdFigSim = figure;
    axis equal; grid on; hold on;
    set(this.hds.hdFigSim, 'Name', 'Simulator', 'NumberTitle', 'off');
    view(2);
    
    % draw map
    this.hds.hdObjMap = plot3(this.map.mks.tvec_w_m(:,1), this.map.mks.tvec_w_m(:,2), this.map.mks.tvec_w_m(:,3), ...
        'o', 'LineWidth', 2, 'MarkerEdgeColor',[0;0;0], 'MarkerFaceColor',[1 0.2 0.2], 'MarkerSize', 10);
    
    % draw base
    this.hds.hdObjBaseX = plot3([pt3d_w_b0(1);pt3d_w_b1(1)], [pt3d_w_b0(2);pt3d_w_b1(2)], [pt3d_w_b0(3);pt3d_w_b1(3)], ...
        'Color', [0 0 1], 'LineWidth', 2);
    this.hds.hdObjBaseY = plot3([pt3d_w_b0(1);pt3d_w_b2(1)], [pt3d_w_b0(2);pt3d_w_b2(2)], [pt3d_w_b0(3);pt3d_w_b2(3)], ...
        'Color', [0 1 0], 'LineWidth', 2);
    this.hds.hdObjBaseZ = plot3([pt3d_w_b0(1);pt3d_w_b3(1)], [pt3d_w_b0(2);pt3d_w_b3(2)], [pt3d_w_b0(3);pt3d_w_b3(3)], ...
        'Color', [1 0 0], 'LineWidth', 2);
    
    % draw camera range
    this.hds.hdObjRange = fill3(X_range,Y_range,Z_range,[1 0 0]);
    alpha(this.hds.hdObjRange, .1);
    
    % set call back function of key pressed 
    set(this.hds.hdFigSim, 'KeyPressFcn', {@this.OnKeyPressed})
    
else
    set(this.hds.hdObjBaseX, 'XData', [pt3d_w_b0(1);pt3d_w_b1(1)]);
    set(this.hds.hdObjBaseX, 'YData', [pt3d_w_b0(2);pt3d_w_b1(2)]);
    set(this.hds.hdObjBaseX, 'ZData', [pt3d_w_b0(3);pt3d_w_b1(3)]);
    
    set(this.hds.hdObjBaseY, 'XData', [pt3d_w_b0(1);pt3d_w_b2(1)]);
    set(this.hds.hdObjBaseY, 'YData', [pt3d_w_b0(2);pt3d_w_b2(2)]);
    set(this.hds.hdObjBaseY, 'ZData', [pt3d_w_b0(3);pt3d_w_b2(3)]);
    
    set(this.hds.hdObjBaseZ, 'XData', [pt3d_w_b0(1);pt3d_w_b3(1)]);
    set(this.hds.hdObjBaseZ, 'YData', [pt3d_w_b0(2);pt3d_w_b3(2)]);
    set(this.hds.hdObjBaseZ, 'ZData', [pt3d_w_b0(3);pt3d_w_b3(3)]);
    
    set(this.hds.hdObjRange, 'XData', X_range);
    set(this.hds.hdObjRange, 'YData', Y_range);
    set(this.hds.hdObjRange, 'ZData', Z_range);
    
%     drawnow;
end

end

