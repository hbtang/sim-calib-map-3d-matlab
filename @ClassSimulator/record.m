function record( this )
%RECORD write into record file Odo.rec and Mk.rec

strOdo = [num2str(this.odo.lp), ' '];
strOdo = [strOdo, num2str(this.odo.lp*0.1), ' ', num2str(this.odo.lp*0.1), ' '];
strOdo = [strOdo, num2str(this.odo.x), ' ',...
    num2str(this.odo.y), ' ', num2str(this.odo.theta), '\n',];
fprintf(this.OutputFileOdoId, strOdo);

if isempty(this.mk.lp)
    return;
end

for i = 1:numel(this.mk.lp)
    lp = this.mk.lp(i);
    id = this.mk.id(i);
    rvec = this.mk.rvec(i,:).';
    tvec = this.mk.tvec(i,:).';
    strMk = [num2str(lp), ' ', num2str(id), ' '];
    strMk = [strMk, num2str(rvec(1)), ' ', num2str(rvec(2)), ' ', ...
        num2str(rvec(3)), ' ',];
    strMk = [strMk, num2str(tvec(1)), ' ', num2str(tvec(2)), ' ', ...
        num2str(tvec(3)), ' ',];
    strMk = [strMk '\n'];
    fprintf(this.OutputFileMkId, strMk);
    
    %% debuging ...
%     ps2d_w_b = [this.odo.x; this.odo.y; this.odo.theta];
%     T3d_w_b = FunPs2d2T3d(ps2d_w_b);
%     T3d_b_c = this.calib.T3d_b_c;
%     T3d_c_m = FunVec2Trans3d(rvec, tvec);
%     T3d_w_m = T3d_w_b*T3d_b_c*T3d_c_m;
%     strMkPrj = [num2str(T3d_w_m(1,4)), ' ', num2str(T3d_w_m(2,4)), ' ', num2str(T3d_w_m(3,4)), ' '];
%     disp(strMkPrj);
end

end

