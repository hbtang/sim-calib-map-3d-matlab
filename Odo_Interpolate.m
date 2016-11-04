function [ odo_ret ] = Odo_Interpolate( odo, odo_raw, time_raw )
% do interpolate odo with the raw data odo_raw

%% compute vx, vy, vtheta

odo_ret = odo;
odo_ret.vx = [];
odo_ret.vy = [];
odo_ret.vtheta = [];

for i = 1:numel(odo.lp)
    lp_i = odo.lp(i);
    row_raw = find(odo_raw.lp == lp_i, 1);
    
    if isempty(row_raw)
        error('Error! FuncIterOdo');
    end
    
    if row_raw == 1
        odo_ret.vx(i,1) = 0;
        odo_ret.vy(i,1) = 0;
        odo_ret.vtheta(i,1) = 0;
    else
        
        dt = time_raw.t_odo(row_raw) - time_raw.t_odo(row_raw-1);
        dt = cnstr2period(dt, 30, -30);
        dx = odo_raw.x(row_raw) - odo_raw.x(row_raw-1);
        dy = odo_raw.y(row_raw) - odo_raw.y(row_raw-1);
        dtheta = odo_raw.theta(row_raw) - odo_raw.theta(row_raw-1);
        dtheta = cnstr2period(dtheta, -pi, pi);
        
        vx = dx/dt;
        vy = dy/dt;
        vtheta = dtheta/dt;
        
        odo_ret.vx(i,1) = vx;
        odo_ret.vy(i,1) = vy;
        odo_ret.vtheta(i,1) = vtheta;        
    end
end

% smooth
odo_ret.vx = smooth(odo_ret.vx, 10);
odo_ret.vy = smooth(odo_ret.vy, 10);
odo_ret.vtheta = smooth(odo_ret.vtheta, 10);

end

