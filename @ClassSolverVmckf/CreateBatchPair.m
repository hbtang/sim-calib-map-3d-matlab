function [ cell_batch ] = CreateBatchPair( this, lp_now, measure, struct_sw )
%CREATEBATCHPAIR Summary of this function goes here
%   Detailed explanation goes here

cell_batch = {};

mk = measure.mk;
vec_lp_sw = struct_sw.vec_lp_sw;

row_mk_now = find(mk.lp == lp_now);
vec_id_now = mk.id(row_mk_now);

%% if vec_lp_sw is empty, set by lp_now
if numel(vec_lp_sw) == 0
    return;
end

%% create sliding window structure
% [~,rows_odo,~] = intersect(odo.lp, vec_lp_sw);
cell_rows_mk = {};
cell_vec_id = {};
for i = 1:numel(vec_lp_sw)
    lp_temp = vec_lp_sw(i);
    rows_mk_temp = find(mk.lp == lp_temp);
    cell_rows_mk{end+1,1} = rows_mk_temp;
    cell_vec_id{end+1,1} = mk.id(rows_mk_temp);
end

%% find lp_sw with same mk observed now
if isempty(vec_id_now)
    return;
end

for i = 1:numel(vec_id_now)
    id_temp = vec_id_now(i);
    vec_lp_temp = [];
    for j = 1:numel(vec_lp_sw);
        if ~isempty(find(cell_vec_id{j} == id_temp, 1))
            vec_lp_temp = [vec_lp_temp; vec_lp_sw(j)];
        end
    end
    if numel(vec_lp_temp) == 0
        continue;
    end
    vec_lp_temp = unique(vec_lp_temp);
    batch_temp = [];
    batch_temp.id = id_temp;
    batch_temp.vec_lp = [vec_lp_temp(end); lp_now];
    if numel(batch_temp.vec_lp) > 1
        cell_batch{end+1, 1} = batch_temp;
    end
end

end

