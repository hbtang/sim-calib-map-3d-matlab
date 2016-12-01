function Run( this )
%RUN main function of simulator

this.Draw;
timestep = 0.1;
lp = 0;

while ~this.flag.bQuit
    %% set loop id
    this.lp = lp;
    
    %% move robot
    ps2d_w_b1 = this.ps2d_w_b;
    ps2d_b1_b2 = [this.vel_lin*timestep; 0; this.vel_rot*timestep];
    ps2d_w_b2 = FunMove2d(ps2d_w_b1, ps2d_b1_b2);
    this.ps2d_w_b = ps2d_w_b2;    
        
    %% refresh mark observed
    this.Observe;
    
    %% refresh raw data
    this.odo_true.lp = [this.odo_true.lp; lp];
    this.odo_true.x = [this.odo_true.x; ps2d_w_b2(1)];
    this.odo_true.y = [this.odo_true.y; ps2d_w_b2(2)];
    this.odo_true.theta = [this.odo_true.theta; ps2d_w_b2(3)];

    this.mk_true.lp = [this.mk_true.lp; this.mk.lp];
    this.mk_true.id = [this.mk_true.id ; this.mk.id];
    this.mk_true.rvec = [this.mk_true.rvec; this.mk.rvec];
    this.mk_true.tvec = [this.mk_true.tvec; this.mk.tvec];
    this.mk_true.image = [this.mk_true.image; this.mk.image];
    
    %% draw results
    this.Draw;
    
    %% print status
    strLp = ['-- lp: ', num2str(lp), '; '];
    strPs = ['-- pose: ', num2str(ps2d_w_b2(1)), ' ', ...
        num2str(ps2d_w_b2(2)), ' ', num2str(ps2d_w_b2(3)), '; '];
    strVel = ['-- vel: ', num2str(this.vel_lin), ' ', ...
        num2str(this.vel_rot), '; '];
    strMk = ['-- mk: ', num2str(numel(this.mk.id)), '; '];
    disp([strLp, strPs, strVel, strMk]);
        
    % end this loop
    lp = lp + 1;
    pause(0.1);   
    
end

end

