clear;

%% init

% path
PathFold = '..\data\sim-sqrmap-inout-20160118\';
PathFoldRes = [PathFold, 'res\'];

OdoNs = 1; MkNs = 1;
InitId = 1; InitNum = 20;
SetId = 1; SetNum = 20;

% flags
FlagDrawInit = 0;
FlagDrawSet = 1;

% ground truth
x_ref = [0;0;1/sqrt(2);-1/sqrt(2);0;0];

%% read final results
MkNsSt = 0; MkNsEnd = 4; MkNsNum = MkNsEnd-MkNsSt;
OdoNsSt = 0; OdoNsEnd = 4; OdoNsNum = OdoNsEnd-OdoNsSt;

% cell_res_final: rowId MkNs, columnId OdoNs
cell_rec_x_final = cell(MkNsEnd - MkNsSt + 1, OdoNsEnd - OdoNsSt + 1);

for MkNs = MkNsSt:MkNsEnd
    for OdoNs = OdoNsSt:OdoNsEnd        
        % rec_x_final_tmp: rowId set\init, columnId states        
        rec_x_final_tmp = zeros(SetNum, 6);
        
        for SetId = 1:SetNum
            
            NameMkStr = ['Mk-z', num2str(MkNs), '-xy', num2str(MkNs)];
            NameOdoStr = ['Odo-l', num2str(OdoNs), '-r', num2str(OdoNs)];
            PathRecData = [PathFoldRes, 'res-',NameMkStr, '-', NameOdoStr,...
                '-s', num2str(SetId), '-i', num2str(1), '.mat'];
            
            load(PathRecData);            
            rec_x_final_tmp(SetId,:) = rec_x(end, :);         
        end
        
        cell_rec_x_final{MkNs-MkNsSt+1, OdoNs-OdoNsSt+1} = rec_x_final_tmp;
        
        % debuging
        disp(['MkNs:',num2str(MkNs), ' OdoNs:', num2str(OdoNs)]);
    end
end

%% draw final results

% generate matrix for boxplot

sz = size(cell_rec_x_final);

mat_q0_ns_s = zeros(SetNum, OdoNsNum*MkNsNum);
mat_q1_ns_s = zeros(SetNum, OdoNsNum*MkNsNum);
mat_q2_ns_s = zeros(SetNum, OdoNsNum*MkNsNum);
mat_q3_ns_s = zeros(SetNum, OdoNsNum*MkNsNum);
mat_x_ns_s = zeros(SetNum, OdoNsNum*MkNsNum);
mat_y_ns_s = zeros(SetNum, OdoNsNum*MkNsNum);

k = 0;
for i = 1:sz(1)
    for j = 1:sz(2)
        k = k+1;
        rec_x_final_tmp = cell_rec_x_final{i,j};
        
        mat_q0_ns_s(:,k) = rec_x_final_tmp(:,1);
        mat_q1_ns_s(:,k) = rec_x_final_tmp(:,2);
        mat_q2_ns_s(:,k) = rec_x_final_tmp(:,3);
        mat_q3_ns_s(:,k) = rec_x_final_tmp(:,4);
        mat_x_ns_s(:,k) = rec_x_final_tmp(:,5);
        mat_y_ns_s(:,k) = rec_x_final_tmp(:,6);        
    end
end

mat_eq0_ns_s = mat_q0_ns_s - x_ref(1);
mat_eq1_ns_s = mat_q1_ns_s - x_ref(2);
mat_eq2_ns_s = mat_q2_ns_s - x_ref(3);
mat_eq3_ns_s = mat_q3_ns_s - x_ref(4);
mat_eqn_ns_s = sqrt(mat_eq0_ns_s.*mat_eq0_ns_s + mat_eq1_ns_s.*mat_eq1_ns_s ...
    + mat_eq2_ns_s.*mat_eq2_ns_s + mat_eq3_ns_s.*mat_eq3_ns_s);

mat_ex_ns_s = mat_x_ns_s - x_ref(5);
mat_ey_ns_s = mat_y_ns_s - x_ref(6);
mat_el_ns_s = sqrt(mat_ex_ns_s.*mat_ex_ns_s + mat_ey_ns_s.*mat_ey_ns_s);

figure;
boxplot(mat_eqn_ns_s); 
set(gca, 'ylim', [0, 0.04]);

figure;
boxplot(mat_el_ns_s); 
set(gca, 'ylim', [0, 500]);

% figure;
% boxplot(mat_q0_ns_s - x_ref(1)); 
% set(gca, 'ylim', [-0.02, 0.02]);
% 
% figure;
% boxplot(mat_q1_ns_s - x_ref(2)); 
% set(gca, 'ylim', [-0.02, 0.02]);
% 
% figure;
% boxplot(mat_q2_ns_s - x_ref(3)); 
% set(gca, 'ylim', [-0.02, 0.02]);
% 
% figure;
% boxplot(mat_q3_ns_s - x_ref(4)); 
% set(gca, 'ylim', [-0.02, 0.02]);
% 
% figure;
% boxplot(mat_x_ns_s - x_ref(5)); 
% set(gca, 'ylim', [-200, 200]);
% 
% figure;
% boxplot(mat_y_ns_s - x_ref(6)); 
% set(gca, 'ylim', [-200, 200]);





