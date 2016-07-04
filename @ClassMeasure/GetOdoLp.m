function odo_out = GetOdoLp(this, lp)
%GETODOLP Summary of this function goes here
%   Detailed explanation goes here
odo_out = [];
row_odo = find(this.odo.lp == lp,1);

if isempty(row_odo)
    return;
end    

odo_out.lp = lp;
odo_out.x = this.odo.x(row_odo);
odo_out.y = this.odo.y(row_odo);
odo_out.theta = this.odo.theta(row_odo);
odo_out.num = 1;

end

