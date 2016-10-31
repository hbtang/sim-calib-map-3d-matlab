clear;

%% init
PathFold = '..\data\sim-sqrmap-inout-20160118\';
PathFoldRes = [PathFold, 'res\'];

% group
OdoNs = 1; MkNs = 1; SetId = 1; InitId = 1;

% FlagShowSet: 1, show different sets; 0, show different inital guess
FlagShowSet = 0;

% number of trials
numTrial = 50;

% state ground truth as referece
x_ref = [0;0;1/sqrt(2);-1/sqrt(2);0;0];

% init variables
recmat_x = cell(6,1);

% plot configuration
FigPos = [1,1,480,480];


%% read data

for i = 1:numTrial
    
    if FlagShowSet == 1
        SetId = i;
    else
        InitId = i;
    end
    
    % set data file path
    NameMkStr = ['Mk-z', num2str(MkNs), '-xy', num2str(MkNs), ...
        ];
    NameOdoStr = ['Odo-l', num2str(OdoNs), '-r', num2str(OdoNs), ...
        ];
    PathRecData = [PathFoldRes, 'res-', NameMkStr, '-', NameOdoStr,...
        '-s', num2str(SetId), '-i', num2str(InitId), '.mat'];
    
    % read data
    load(PathRecData);
    
    recmat_x{1} = [recmat_x{1} rec_x(:,1)];
    recmat_x{2} = [recmat_x{2} rec_x(:,2)];
    recmat_x{3} = [recmat_x{3} rec_x(:,3)];
    recmat_x{4} = [recmat_x{4} rec_x(:,4)];
    recmat_x{5} = [recmat_x{5} rec_x(:,5)];
    recmat_x{6} = [recmat_x{6} rec_x(:,6)];
    
end

%% calculate RMSE

% mat_err_rot = zeros(size(recmat_x{1}));
% mat_err_trans = zeros(size(recmat_x{1}));

% aRMSE
mat_err_q0 = recmat_x{1} - x_ref(1);
mat_err_q1 = recmat_x{2} - x_ref(2);
mat_err_q2 = recmat_x{3} - x_ref(3);
mat_err_q3 = recmat_x{4} - x_ref(4);
mat_err_x = recmat_x{5} - x_ref(5);
mat_err_y = recmat_x{6} - x_ref(6);

mat_err_rot = sqrt(mat_err_q0.*mat_err_q0 + mat_err_q1.*mat_err_q1 ...
    + mat_err_q2.*mat_err_q2 + mat_err_q3.*mat_err_q3);
mat_err_trans = sqrt(mat_err_x.*mat_err_x + mat_err_y.*mat_err_y);

vec_armse_rot = rms(mat_err_rot.');
vec_armse_trans = rms(mat_err_trans.');

% rRMSE
mat_rerr_q0 = recmat_x{1} - repmat(mean(recmat_x{1}.').',1,50);
mat_rerr_q1 = recmat_x{2} - repmat(mean(recmat_x{2}.').',1,50);
mat_rerr_q2 = recmat_x{3} - repmat(mean(recmat_x{3}.').',1,50);
mat_rerr_q3 = recmat_x{4} - repmat(mean(recmat_x{4}.').',1,50);
mat_rerr_x = recmat_x{5} - repmat(mean(recmat_x{5}.').',1,50);
mat_rerr_y = recmat_x{6} - repmat(mean(recmat_x{6}.').',1,50);

mat_rerr_rot = sqrt(mat_rerr_q0.*mat_rerr_q0 + mat_rerr_q1.*mat_rerr_q1 ...
    + mat_rerr_q2.*mat_rerr_q2 + mat_rerr_q3.*mat_rerr_q3);
mat_rerr_trans = sqrt(mat_rerr_x.*mat_rerr_x + mat_rerr_y.*mat_rerr_y);

vec_rrmse_rot = rms(mat_rerr_rot.');
vec_rrmse_trans = rms(mat_rerr_trans.');

%% draw and save
numJump = 30;
lpTailSt = 350;
numJumpTail = 10;

if FlagShowSet == 1
    strPrefix = ['Mk-', num2str(MkNs), '-Odo-', num2str(OdoNs), '-s'];
else
    strPrefix = ['Mk-', num2str(MkNs), '-Odo-', num2str(OdoNs), '-i'];
end

% rotation
fig = figure; 
fig.Position = FigPos;

subplot(2,1,1);
hold on; grid on;

boxplot(mat_err_rot(1:numJump:end,:).', floor(rec_lp(1:numJump:end)/30));
plot(vec_armse_rot(1:numJump:end).', '--', 'Color', 'b');
plot(vec_rrmse_rot(1:numJump:end).', '--', 'Color', 'r');

ax = gca; ax.XTickLabelRotation = 45;
xlabel('Time (sec)'); 
ylabel('Rot. Err. (rad)');
legend({'aRMSE','rRMSE'});
title(['Calib. Res.: ', 'MkNs=', num2str(MkNs), '%, OdoNs', num2str(OdoNs), '%'],...
    'FontWeight', 'bold');

% rotation tail
subplot(2,1,2);
hold on; grid on;

fig.Position = FigPos;

boxplot(mat_err_rot(lpTailSt:numJumpTail:end,:).', floor(rec_lp(lpTailSt:numJumpTail:end)/30));
plot(vec_armse_rot(lpTailSt:numJumpTail:end).', '--', 'Color', 'b');
plot(vec_rrmse_rot(lpTailSt:numJumpTail:end).', '--', 'Color', 'r');

ax = gca; ax.XTickLabelRotation = 45;
xlabel('Time (sec)'); ylabel('Rot. Err. (rad)');
legend({'aRMSE','rRMSE'});

set(gcf, 'PaperPositionMode', 'auto');
print(['./temp/', strPrefix, '-ErrRot-t'],'-depsc','-r0');
print(['./temp/', strPrefix, '-ErrRot-t'],'-dmeta','-r0');
print(['./temp/', strPrefix, '-ErrRot-t'],'-djpeg','-r0');

% translation
fig = figure; 
fig.Position = FigPos;

subplot(2,1,1);
hold on; grid on;

boxplot(mat_err_trans(1:numJump:end,:).', floor(rec_lp(1:numJump:end)/30));
plot(vec_armse_trans(1:numJump:end).', '--', 'Color', 'b');
plot(vec_rrmse_trans(1:numJump:end).', '--', 'Color', 'r');

ax = gca; ax.XTickLabelRotation = 45;
xlabel('Time (sec)'); 
ylabel('Trans. Err. (mm)');
legend({'aRMSE','rRMSE'});
title(['Calib. Res.: ', 'MkNs=', num2str(MkNs), '%, OdoNs', num2str(OdoNs), '%'],...
    'FontWeight', 'bold');

% translation tail

subplot(2,1,2);
hold on; grid on;

boxplot(mat_err_trans(lpTailSt:numJumpTail:end,:).', floor(rec_lp(lpTailSt:numJumpTail:end)/30));
plot(vec_armse_trans(lpTailSt:numJumpTail:end).', '--', 'Color', 'b');
plot(vec_rrmse_trans(lpTailSt:numJumpTail:end).', '--', 'Color', 'r');

ax = gca; ax.XTickLabelRotation = 45;
xlabel('Time (sec)'); ylabel('Trans. Err. (mm)');
legend({'aRMSE','rRMSE'});

set(gcf, 'PaperPositionMode', 'auto');
print(['./temp/', strPrefix, '-ErrTrans-t'],'-depsc','-r0');
print(['./temp/', strPrefix, '-ErrTrans-t'],'-dmeta','-r0');
print(['./temp/', strPrefix, '-ErrTrans-t'],'-djpeg','-r0');



