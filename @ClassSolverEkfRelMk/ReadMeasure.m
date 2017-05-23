function [b_read_fail, struct_measure] = ReadMeasure(this, measure)
%READMEASURE 此处显示有关此函数的摘要
%   此处显示详细说明

%% init

b_read_fail = false;
struct_measure = [];

this.lp_last = this.lp_now;
lp_last = this.lp_last;

row_lplast_msrodolp = find(measure.odo.lp == lp_last, 1);

% error in lp_last, return
if isempty(row_lplast_msrodolp)
    error('Error in ReadMeasure!');
end

% last record, return
if row_lplast_msrodolp == numel(measure.odo.lp)
    b_read_fail = true;
    return;
end

%% read data
row1 = row_lplast_msrodolp;
row2 = row_lplast_msrodolp+1;

% read lp
lp_now = measure.odo.lp(row2);

% read odo
se2_o_b1 = [measure.odo.x(row1);measure.odo.y(row1);measure.odo.theta(row1)];
se2_o_b2 = [measure.odo.x(row2);measure.odo.y(row2);measure.odo.theta(row2)];
se2_b1_b2 = FunRelPos2d( se2_o_b1, se2_o_b2 );
odo.se2_o_b1 = se2_o_b1;
odo.se2_o_b2 = se2_o_b2;
odo.se2_b1_b2 = se2_b1_b2;

% read mk
rowmask_now_msrmk = measure.mk.lp == lp_now;
mk.id = measure.mk.id(rowmask_now_msrmk);
mk.rvec = measure.mk.rvec(rowmask_now_msrmk,:);
mk.tvec = measure.mk.tvec(rowmask_now_msrmk,:);

%% output
struct_measure.lp_now = lp_now;
struct_measure.lp_last = lp_last;
struct_measure.odo = odo;
struct_measure.mk = mk;

this.lp_now = lp_now;
this.lp_last = lp_last;

end

