function struct_ret = Err_vSlam( measure, calib, map, setting, options )
% Compute error vector of both mark measurements and odometry measurement

if nargin < 5
    options = [];
end
if ~isfield(options, 'bCalibTmp')
    options.bCalibTmp = false;
end
if ~isfield(options, 'bCalibOdo')
    options.bCalibOdo = false;
end

mat_camera = calib.mat_camera;
vec_distortion = calib.vec_distortion;

%% init data
mk = measure.mk;
odo = measure.odo;
time = measure.time;

T3d_b_c = calib.T3d_b_c;
T3d_c_b = inv(T3d_b_c);

if options.bCalibTmp    
    dt_b_c = calib.dt;
else
    dt_b_c = 0;
end

if options.bCalibOdo
    k_odo_lin = calib.k_odo_lin;
    k_odo_rot = calib.k_odo_rot;
else
    k_odo_lin = 1;
    k_odo_rot = 1;
end

vecDt = zeros(numel(time.lp), 1);
for i = 1:numel(time.lp)
    dtTmp = cnstr2period(time.t_mk(i) - time.t_odo(i), 30, -30);
    vecDt(i,1) = dtTmp;
end

mat_ps2d_w_b = map.kfs.ps2d_w_b;
mat_rvec_w_m = map.mks.rvec_w_m;
mat_tvec_w_m = map.mks.tvec_w_m;

tvec_m_pt1 = setting.aruco.tvec_m_pt1.';
tvec_m_pt2 = setting.aruco.tvec_m_pt2.';
tvec_m_pt3 = setting.aruco.tvec_m_pt3.';
tvec_m_pt4 = setting.aruco.tvec_m_pt4.';

%% mark observation part
mat_errImg = zeros(mk.num, 8);
for i = 1:mk.num
 
    lp = mk.lp(i);
    row_odo = find(odo.lp == lp,1);
    x_w_b = mat_ps2d_w_b(row_odo,1);
    y_w_b = mat_ps2d_w_b(row_odo,2);
    theta_w_b = mat_ps2d_w_b(row_odo,3);
    rvec_w_b = [0; 0; theta_w_b];
    tvec_w_b = [x_w_b; y_w_b; 0];
    [T3d_w_b, R3d_w_b] = FunVec2Trans3d(rvec_w_b, tvec_w_b);
    T3d_b_w = [R3d_w_b.' -R3d_w_b.'*tvec_w_b; 0 0 0 1];
    
    mkId = mk.id(i);
    mkIdOrd = find(mk.vecMkId(:,1) == mkId, 1);
    
    rvec_w_m = mat_rvec_w_m(mkIdOrd,:).';
    tvec_w_m = mat_tvec_w_m(mkIdOrd,:).';
    T3d_w_m = FunVec2Trans3d(rvec_w_m, tvec_w_m);
    
    T3d_c_m = T3d_c_b*T3d_b_w*T3d_w_m;
    
    [ rvec_c_m, tvec_c_m ] = FunTrans2Vec3d( T3d_c_m );
    
    img_c_pt1_measure = mk.pt1(i,:).';
    img_c_pt2_measure = mk.pt2(i,:).';
    img_c_pt3_measure = mk.pt3(i,:).';
    img_c_pt4_measure = mk.pt4(i,:).';
    
    img_c_pt1 = ProjXYZ2UV( rvec_c_m, tvec_c_m, tvec_m_pt1, mat_camera, vec_distortion );
    img_c_pt2 = ProjXYZ2UV( rvec_c_m, tvec_c_m, tvec_m_pt2, mat_camera, vec_distortion );
    img_c_pt3 = ProjXYZ2UV( rvec_c_m, tvec_c_m, tvec_m_pt3, mat_camera, vec_distortion );
    img_c_pt4 = ProjXYZ2UV( rvec_c_m, tvec_c_m, tvec_m_pt4, mat_camera, vec_distortion );
    
    errimg_c_pt1 = img_c_pt1 - img_c_pt1_measure;
    errimg_c_pt2 = img_c_pt2 - img_c_pt2_measure;
    errimg_c_pt3 = img_c_pt3 - img_c_pt3_measure;
    errimg_c_pt4 = img_c_pt4 - img_c_pt4_measure;
    
    vec_errImg = [errimg_c_pt1; errimg_c_pt2; errimg_c_pt3; errimg_c_pt4];
    mat_errImg(i,:) = vec_errImg.';
end

%% odometry part
mat_errOdo = zeros(odo.num-1, 3);
for i = 2:odo.num
    if options.bCalibTmp
        dt_b1_c1 = (vecDt(i-1) + dt_b_c);
        dt_b2_c2 = (vecDt(i) + dt_b_c);
    else
        dt_b1_c1 = 0;
        dt_b2_c2 = 0;
    end
    
    ps2d_w_b1_odo = [odo.x(i-1);odo.y(i-1);odo.theta(i-1)];
    ps2d_w_b1t_odo = ps2d_w_b1_odo + ...
        dt_b1_c1*[odo.vx(i-1);odo.vy(i-1);odo.vtheta(i-1)];
    ps2d_w_b2_odo = [odo.x(i);odo.y(i);odo.theta(i)];
    ps2d_w_b2t_odo = ps2d_w_b2_odo + ...
        dt_b2_c2*[odo.vx(i);odo.vy(i);odo.vtheta(i)];
    
    ps2d_b1_b2_odo = FunRelPos2d(ps2d_w_b1t_odo, ps2d_w_b2t_odo);
    ps2d_b1_b2_odo(1:2) = k_odo_lin*ps2d_b1_b2_odo(1:2);
    ps2d_b1_b2_odo(3) = k_odo_rot*ps2d_b1_b2_odo(3);
    
    x_w_b1 = mat_ps2d_w_b(i-1,1);
    y_w_b1 = mat_ps2d_w_b(i-1,2);
    theta_w_b1 = mat_ps2d_w_b(i-1,3);
    
    x_w_b2 = mat_ps2d_w_b(i,1);
    y_w_b2 = mat_ps2d_w_b(i,2);
    theta_w_b2 = mat_ps2d_w_b(i,3);
    
    ps2d_w_b1 = [x_w_b1; y_w_b1; theta_w_b1];
    ps2d_w_b2 = [x_w_b2; y_w_b2; theta_w_b2];
    ps2d_b1_b2_map = FunRelPos2d(ps2d_w_b1, ps2d_w_b2);
    ps2d_b1_b2_map(3) = FunPrdCnst(ps2d_b1_b2_map(3), ps2d_b1_b2_odo+pi, ps2d_b1_b2_odo-pi);
    
    errPs2d_b1_b2 = ps2d_b1_b2_odo - ps2d_b1_b2_map;
    
    mat_errOdo(i-1,:) = errPs2d_b1_b2.';
    
    %     err_OdoNormal(i-1,:) = (sqrt(CovOdo( ps2d_b1_b2_odo, errConfig ))\errPs2d_b1_b2).';
end

struct_ret.mat_errOdo = mat_errOdo;
struct_ret.mat_errImg = mat_errImg;

end

