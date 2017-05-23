clear;

%% init
PathFoldRes = 'C:\Workspace\Data\TRO\exp-vm-2\';
OdoNs = 1; MkNs = 1; SetId = 1;
MethodTitle = 'VMCGF';

% FlagShowSet: 1, show different sets; 0, show different inital guess
FlagShowSet = 0;

% number of trials
numTrial = 20;

% init variables
recmat_x = cell(5,1);

% plot configuration
FigPos = [1,1,480,200];


%% read data
mat_err_rx = [];
mat_err_ry = [];
mat_err_rz = [];
mat_err_x = [];
mat_err_y = [];

mat_rx = [];
mat_ry = [];
mat_rz = [];
mat_x = [];
mat_y = [];

for i = 1:numTrial
    
    SetId = i;
    
    % set data file path
    NameMkStr = ['Mk-z', num2str(MkNs), '-xy', num2str(MkNs), ...
        ];
    NameOdoStr = ['Odo-l', num2str(OdoNs), '-r', num2str(OdoNs), ...
        ];
    PathRecData = [PathFoldRes, 'Rec-s', num2str(SetId), '.mat'];
    
    % read data
    load(PathRecData);
    
%     recmat_x{1} = [recmat_x{1} rec_rvec(:,1)];
%     recmat_x{2} = [recmat_x{2} rec_rvec(:,2)];
%     recmat_x{3} = [recmat_x{3} rec_rvec(:,3)];
%     recmat_x{4} = [recmat_x{4} rec_tvec(:,1)];
%     recmat_x{5} = [recmat_x{5} rec_tvec(:,2)]; 

    mat_err_rx = [mat_err_rx, rec_drvec(:,1)];
    mat_err_ry = [mat_err_ry, rec_drvec(:,2)];
    mat_err_rz = [mat_err_rz, rec_drvec(:,3)];
    mat_err_x = [mat_err_x, rec_dtvec(:,1)];
    mat_err_y = [mat_err_y, rec_dtvec(:,2)];
    
    mat_rx = [mat_rx, rec_rvec(:,1)];
    mat_ry = [mat_ry, rec_rvec(:,2)];
    mat_rz = [mat_rz, rec_rvec(:,3)];
    mat_x = [mat_x, rec_tvec(:,1)];
    mat_y = [mat_y, rec_tvec(:,2)];

end

%% calculate RMSE

% mat_err_rot = zeros(size(recmat_x{1}));
% mat_err_trans = zeros(size(recmat_x{1}));

% aRMSE
mat_err_rot = sqrt(mat_err_rx.*mat_err_rx + mat_err_ry.*mat_err_ry ...
    + mat_err_rz.*mat_err_rz);
mat_err_trans = sqrt(mat_err_x.*mat_err_x + mat_err_y.*mat_err_y);
vec_armse_rot = rms(mat_err_rot.');
vec_armse_trans = rms(mat_err_trans.');
vec_armse_rx = rms(mat_err_rx.');
vec_armse_ry = rms(mat_err_ry.');
vec_armse_rz = rms(mat_err_rz.');
vec_armse_x = rms(mat_err_x.');
vec_armse_y = rms(mat_err_y.');


%% draw and save
numJump = 30;
lpTailSt = 500;
numJumpTail = 15;

if FlagShowSet == 1
    strPrefix = ['Mk-', num2str(MkNs), '-Odo-', num2str(OdoNs), '-s'];
else
    strPrefix = ['Mk-', num2str(MkNs), '-Odo-', num2str(OdoNs), '-i'];
end

% rotation
fig1 = figure; 
fig1.Position = FigPos;
% subplot(2,1,1);
hold on; grid on;
boxplot(mat_err_rot(1:numJump:end,:).', floor(vec_lp(1:numJump:end)/30));
plot(vec_armse_rot(1:numJump:end).', '--', 'Color', 'b');
% plot(vec_rrmse_rot(1:numJump:end).', '--', 'Color', 'r');
ax = gca; ax.XTickLabelRotation = 45;
xlabel('Time (sec)'); 
ylabel('Rot. Err. (rad)');
legend({'aRMSE'});
set(gcf, 'PaperPositionMode', 'auto');
title([MethodTitle, '. Res.: AGV Dataset'],...
    'FontWeight', 'bold');
print(['./temp/', 'ErrRot-t'],'-depsc','-r0');
print(['./temp/', 'ErrRot-t'],'-dmeta','-r0');
print(['./temp/', 'ErrRot-t'],'-djpeg','-r0');

% rotation tail
% subplot(2,1,2);
fig2 = figure; 
fig2.Position = FigPos;
hold on; grid on;
boxplot(mat_err_rot(lpTailSt:numJumpTail:end,:).', floor(vec_lp(lpTailSt:numJumpTail:end)/30));
plot(vec_armse_rot(lpTailSt:numJumpTail:end).', '--', 'Color', 'b');
% plot(vec_rrmse_rot(lpTailSt:numJumpTail:end).', '--', 'Color', 'r');
ax = gca; ax.XTickLabelRotation = 45;
xlabel('Time (sec)'); ylabel('Rot. Err. (rad)');
legend({'aRMSE'});
set(gcf, 'PaperPositionMode', 'auto');
title([MethodTitle, '. Res.: AGV Dataset'],...
    'FontWeight', 'bold');
print(['./temp/', 'ErrRot-t-tail'],'-depsc','-r0');
print(['./temp/', 'ErrRot-t-tail'],'-dmeta','-r0');
print(['./temp/', 'ErrRot-t-tail'],'-djpeg','-r0');

% translation
fig3 = figure; 
fig3.Position = FigPos;
% subplot(2,1,1);
hold on; grid on;
boxplot(mat_err_trans(1:numJump:end,:).', floor(vec_lp(1:numJump:end)/30));
plot(vec_armse_trans(1:numJump:end).', '--', 'Color', 'b');
% plot(vec_rrmse_trans(1:numJump:end).', '--', 'Color', 'r');
ax = gca; ax.XTickLabelRotation = 45;
xlabel('Time (sec)'); 
ylabel('Trans. Err. (mm)');
legend({'aRMSE'});
title([MethodTitle, '. Res.: AGV Dataset'],...
    'FontWeight', 'bold');
set(gcf, 'PaperPositionMode', 'auto');
print(['./temp/', 'ErrTrans-t'],'-depsc','-r0');
print(['./temp/', 'ErrTrans-t'],'-dmeta','-r0');
print(['./temp/', 'ErrTrans-t'],'-djpeg','-r0');

% translation tail
% subplot(2,1,2);
fig4 = figure; 
fig4.Position = FigPos;
hold on; grid on;
boxplot(mat_err_trans(lpTailSt:numJumpTail:end,:).', floor(vec_lp(lpTailSt:numJumpTail:end)/30));
plot(vec_armse_trans(lpTailSt:numJumpTail:end).', '--', 'Color', 'b');
% plot(vec_rrmse_trans(lpTailSt:numJumpTail:end).', '--', 'Color', 'r');
ax = gca; ax.XTickLabelRotation = 45;
xlabel('Time (sec)'); ylabel('Trans. Err. (mm)');
legend({'aRMSE'});
title([MethodTitle, '. Res.: AGV Dataset'],...
    'FontWeight', 'bold');
set(gcf, 'PaperPositionMode', 'auto');
print(['./temp/', 'ErrTrans-t-tail'],'-depsc','-r0');
print(['./temp/', 'ErrTrans-t-tail'],'-dmeta','-r0');
print(['./temp/', 'ErrTrans-t-tail'],'-djpeg','-r0');

%% print final rmse
disp(['rmse.rx=', num2str(vec_armse_rx(end))]);
disp(['rmse.ry=', num2str(vec_armse_ry(end))]);
disp(['rmse.rz=', num2str(vec_armse_rz(end))]);
disp(['rmse.x=', num2str(vec_armse_x(end))]);
disp(['rmse.y=', num2str(vec_armse_y(end))]);


disp(['mean.rx=', num2str(mean(mat_rx(end,:)))]);
disp(['mean.ry=', num2str(mean(mat_ry(end,:)))]);
disp(['mean.rz=', num2str(mean(mat_rz(end,:)))]);
disp(['mean.x=', num2str(mean(mat_x(end,:)))]);
disp(['mean.y=', num2str(mean(mat_y(end,:)))]);


