function RefreshSlidingWindow( this, lp_now, measure )
%REFRESHSLIDINGWINDOW: refresh this.vec_lp_sw, add lp_now, and remain last
%mk observation or same with mk_now

odo = measure.odo;
mk = measure.mk;
vec_lp_sw = this.vec_lp_sw;

row_mk_now = find(mk.lp == lp_now);
vec_mkId_now = mk.id(row_mk_now);

%% if vec_lp_sw is empty, set by lp_now
if numel(vec_lp_sw) == 0
    rows_mk_now = find(mk.lp == lp_now);
    if numel(rows_mk_now) > 0
        this.vec_lp_sw = lp_now;
    end
    return;
end

%% create sliding window structure
[~,rows_odo,~] = intersect(odo.lp, vec_lp_sw);
cell_rows_mk = {};
cell_vec_mkId = {};
for i = 1:numel(vec_lp_sw)
    lp_temp = vec_lp_sw(i);
    rows_mk_temp = find(mk.lp == lp_temp);
    cell_rows_mk{end+1,1} = rows_mk_temp;
    cell_vec_mkId{end+1,1} = mk.id(rows_mk_temp);
end

%% find lp_sw with same mk observed now
vec_lp_sw_1 = [];
for i = 1:numel(vec_lp_sw)
    lp_temp = vec_lp_sw(i);
    vec_temp = intersect(vec_mkId_now, cell_vec_mkId{i});
    if ~isempty(vec_temp)
        vec_lp_sw_1 = [vec_lp_sw_1; lp_temp];
    end
end

%% find last lp for each mk
vec_mkId_sorted = [];
vec_lp_sw_2 = [];
for i = numel(vec_lp_sw):-1:1
    lp_temp = vec_lp_sw(i);
    for j = 1:numel(cell_vec_mkId{i})
        mkId_temp = cell_vec_mkId{i}(j);
        if isempty(find(vec_mkId_sorted == mkId_temp))
            vec_lp_sw_2 = [vec_lp_sw_2; lp_temp];
            vec_mkId_sorted = unique([vec_mkId_sorted; cell_vec_mkId{i}]);
            continue;
        end       
    end    
end

%% return

this.vec_lp_sw = unique([vec_lp_sw_1; vec_lp_sw_2; lp_now]);

end

