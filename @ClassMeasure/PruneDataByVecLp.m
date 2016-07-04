function PruneDataByVecLp( this, vecLp )
%PRUNEDATA prune raw input data


%% prune odometry data

odo_new.lp = this.odo.lp(vecLp+1);
odo_new.x = this.odo.x(vecLp+1);
odo_new.y = this.odo.y(vecLp+1);
odo_new.theta = this.odo.theta(vecLp+1);
odo_new.num = numel(this.odo.lp);
this.odo = odo_new;
disp(['Odometry info prunning is done, ', num2str(odo_new.num), ' records remained.']);

%% prune mark data
mk_new = struct('lp', [], 'id', [], ...
    'rvec', [], 'tvec', [], 'num', [], ...
    'numMkId', [], 'vecMkId', []);
for i = 1:this.mk.num
    tmp = find(this.odo.lp == this.mk.lp(i), 1);
    if numel(tmp) ~= 0
        mk_new.lp = [mk_new.lp; this.mk.lp(i)];
        mk_new.id = [mk_new.id; this.mk.id(i)];
        mk_new.rvec = [mk_new.rvec; this.mk.rvec(i,:)];
        mk_new.tvec = [mk_new.tvec; this.mk.tvec(i,:)];
    end
end
mk_new.num = numel(mk_new.lp);
mk_new.vecMkId = unique(mk_new.id);
mk_new.numMkId = numel(mk_new.vecMkId);
this.mk = mk_new;
disp(['Marker observation info prunning is done, ', num2str(mk_new.num), ' records remained.'])
disp(' ');

end


