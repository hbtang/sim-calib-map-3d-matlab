function [struct_measure] = ClearMkOld(this, struct_measure)
%CLEARMKOLD clear all info related to the old marks
%   for ekf without loop closure

mk_now = struct_measure.mk;
num_mk_old = numel(this.vec_mkid);

% find old marks
vec_mkid_del = [];
rows_mkid_del = [];
rows_vecmux_del = [];
for i = 1:num_mk_old
    mkid_old = this.vec_mkid(i);
    rows_vecmux_i = (9+3*i-2:9+3*i).';
    if any(mk_now.id == mkid_old)
        continue;
    else
        vec_mkid_del = [vec_mkid_del; mkid_old];
        rows_mkid_del = [rows_mkid_del; i];
        rows_vecmux_del = [rows_vecmux_del; rows_vecmux_i];
    end
end

% do clear
this.vec_mu_x(rows_vecmux_del) = [];
this.mat_Sigma_x(rows_vecmux_del,:) = [];
this.mat_Sigma_x(:,rows_vecmux_del) = [];
this.vec_mkid(rows_mkid_del) = [];

end

