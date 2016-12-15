function SolveYawXYGuo(this, measure, calib)
%SOLVEYAWXYGUO 此处显示有关此函数的摘要
%   此处显示详细说明
disp('Init: solve yaw and xy by VO based solution ...');

%% build constraints
disp('Building constraints ...');

odo = measure.odo;
mk = measure.mk;

threshLpLocal = 10;

cell_cnstr = {};
for i = 1:numel(mk.vecMkId)
    mkId_i = mk.vecMkId(i);
    vecMkFind = find(mk.id == mkId_i);
    vecMkLpFind = mk.lp(vecMkFind);
    
    for j = 2:numel(vecMkLpFind)
        lp1 = vecMkLpFind(j-1);
        lp2 = vecMkLpFind(j);
        
        if abs(lp2 - lp1) > threshLpLocal
            continue;
        end
        
        rowOdo1 = find(odo.lp == lp1);
        rowOdo2 = find(odo.lp == lp2);
        rowMk1 = vecMkFind(j-1);
        rowMk2 = vecMkFind(j);
        
        cnstr.rowOdo1 = rowOdo1;
        cnstr.rowOdo2 = rowOdo2;
        cnstr.rowMk1 = rowMk1;
        cnstr.rowMk2 = rowMk2;
        
        cell_cnstr{end+1} = cnstr;
    end
end

%% step 1: init beta-gamma in ZYZ Euler angle alpha-beta-gamma

T3d_cg_c = calib.T3d_cg_c;
R3d_cg_c = T3d_cg_c(1:3,1:3);
R3_b_c_betagamma = R3d_cg_c;

%% step 2: init remaining unknowns
disp('Init remaining unknowns...');
numCnstr = numel(cell_cnstr);
G = zeros(2*numCnstr, 4);
w = zeros(2*numCnstr, 1);

%%% debug ...
% rec_rvec_b2_b1 = [];
% rec_tvec_b2_b1 = [];
% rec_rvec_c2_c1 = [];
% rec_tvec_c2_c1 = [];
%%% debug end.

for i = 1:numCnstr
    
    rowOdo1 = cell_cnstr{i}.rowOdo1;
    rowOdo2 = cell_cnstr{i}.rowOdo2;
    rowMk1 = cell_cnstr{i}.rowMk1;
    rowMk2 = cell_cnstr{i}.rowMk2;
    
    x_o_b1 = odo.x(rowOdo1);
    y_o_b1 = odo.y(rowOdo1);
    theta_o_b1 = odo.theta(rowOdo1);
    x_o_b2 = odo.x(rowOdo2);
    y_o_b2 = odo.y(rowOdo2);
    theta_o_b2 = odo.theta(rowOdo2);
    
    tvec_o_b1 = [x_o_b1; y_o_b1; 0];
    rvec_o_b1 = [0; 0; theta_o_b1];
    T3_o_b1 = FunVec2Trans3d(rvec_o_b1, tvec_o_b1);
    tvec_o_b2 = [x_o_b2; y_o_b2; 0];
    rvec_o_b2 = [0; 0; theta_o_b2];
    T3_o_b2 = FunVec2Trans3d(rvec_o_b2, tvec_o_b2);
    T3_b2_b1 = inv(T3_o_b2)*T3_o_b1;
    [ rvec_b2_b1, tvec_b2_b1 ] = FunTrans2Vec3d( T3_b2_b1 );
    
    dtheta_b2_b1 = cnstr2period(theta_o_b1 - theta_o_b2, pi, -pi);
    
    rvec_c1_m = mk.rvec(rowMk1,:).';
    tvec_c1_m = mk.tvec(rowMk1,:).';
    rvec_c2_m = mk.rvec(rowMk2,:).';
    tvec_c2_m = mk.tvec(rowMk2,:).';
    T3_c1_m = FunVec2Trans3d( rvec_c1_m, tvec_c1_m );
    T3_c2_m = FunVec2Trans3d( rvec_c2_m, tvec_c2_m );
    T3_c2_c1 = T3_c2_m*inv(T3_c1_m);
    [ rvec_c2_c1, tvec_c2_c1 ] = FunTrans2Vec3d( T3_c2_c1 );
    
    % follow eq. 22
    phi_i = dtheta_b2_b1;
    J_i = [cos(phi_i)-1 -sin(phi_i); sin(phi_i) cos(phi_i)-1];
    p_i = R3_b_c_betagamma*tvec_c2_c1;
    p_i_1 = p_i(1);
    p_i_2 = p_i(2);
    K_i = -[p_i_1 -p_i_2; p_i_2 p_i_1];
    t_i = tvec_b2_b1(1:2);
    
    G(2*i-1:2*i, :) = [J_i K_i];
    w(2*i-1:2*i, :) = t_i;
    
    %%% debug ...
    %     rec_rvec_b2_b1 = [rec_rvec_b2_b1; rvec_b2_b1.'];
    %     rec_tvec_b2_b1 = [rec_tvec_b2_b1; tvec_b2_b1.'];
    %     rec_rvec_c2_c1 = [rec_rvec_c2_c1; rvec_c2_c1.'];
    %     rec_tvec_c2_c1 = [rec_tvec_c2_c1; tvec_c2_c1.'];
    %%% debug end.
end

m_opt = -pinv(G)*w;
x_b_c = m_opt(1);
y_b_c = m_opt(2);
tvec_b_c = [x_b_c; y_b_c; 0];
alpha_b_c = atan2(m_opt(4), m_opt(3));
rvec_b_c_alpha = [0; 0; alpha_b_c];
R3_b_c_alpha = rodrigues(rvec_b_c_alpha);
R3_b_c = R3_b_c_alpha*R3_b_c_betagamma;
rvec_b_c = rodrigues(R3_b_c);

%% set calib
disp('Init: solve yaw and xy by VO based solution done!');
calib.SetVecbc(rvec_b_c, tvec_b_c);