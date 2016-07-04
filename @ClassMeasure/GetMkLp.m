function mk_out = GetMkLp(this, lp)
%GETMKLP Summary of this function goes here
%   Detailed explanation goes here
mk_out = [];
row_mk = find(this.mk.lp == lp);

if isempty(row_mk)
    return;
end    

mk_out.lp = this.mk.lp(row_mk);
mk_out.id = this.mk.id(row_mk);
mk_out.rvec = this.mk.rvec(row_mk,:);
mk_out.tvec = this.mk.tvec(row_mk,:);
mk_out.num = numel(row_mk);
mk_out.numMkId = numel(row_mk);
mk_out.vecMkId = mk_out.id;

end

