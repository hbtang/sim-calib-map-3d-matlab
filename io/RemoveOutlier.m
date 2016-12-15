function [ mat_data_ret ] = RemoveOutlier( mat_data, flag, config )
%REMOVEOUTLIER ratio

[numrow, numcol] = size(mat_data);

switch flag
    case 1
        vec_maen = mean(mat_data);
        vec_std = std(mat_data);
        ratio = config;
        vec_upperbound = vec_maen + vec_std*ratio;
        vec_lowerbound = vec_maen - vec_std*ratio;
    case 2
        vec_upperbound = zeros(1,numcol);
        vec_lowerbound = zeros(1,numcol);
        ratio = config;
        for i = 1:numcol
            vec_data = mat_data(:,i);
            vec_data_sort = sort(vec_data);
            idx_middle = floor(0.5*numrow);
            idx_lower = floor(0.25*numrow);
            idx_upper = floor(0.75*numrow);
            val_middle = vec_data_sort(idx_middle);
            val_lower = vec_data_sort(idx_lower);
            val_upper = vec_data_sort(idx_upper);
            val_lowerbound = val_middle + (val_lower-val_middle)*ratio;
            val_upperbound = val_middle + (val_upper-val_middle)*ratio;
            vec_lowerbound(i) = val_lowerbound;
            vec_upperbound(i) = val_upperbound;
        end
    case 3
        vec_upperbound = config(1,:);
        vec_lowerbound = config(2,:);
end

vec_idxgood = [];
for i = 1:numrow
    b_good = true;
    for j = 1:numcol
        if mat_data(i,j) > vec_upperbound(j) || ...
                mat_data(i,j) < vec_lowerbound(j)
            b_good = false;
        end
    end
    if b_good
        vec_idxgood = [vec_idxgood; i];
    end
end

mat_data_ret = mat_data(vec_idxgood,:);
end

