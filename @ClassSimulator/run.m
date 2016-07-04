function run( this )
%RUN main function of simulator

this.draw;
timeStep = 0.1;
lp = 0;

while ~this.ifQuit
    % set loop id
    this.lp = lp;
    
    % move robot
    ps2d_w_b1 = this.ps2d_w_b;
    ps2d_b1_b2 = [this.velLin*timeStep; 0; this.velRot*timeStep];
    ps2d_w_b2 = FunMove2d(ps2d_w_b1, ps2d_b1_b2);
    this.ps2d_w_b = ps2d_w_b2;
    
    % refresh odometry measurement
    ps2d_o_b1 = [this.odo.x; this.odo.y; this.odo.theta];
    ps2d_b1_b2_odo = ps2d_b1_b2;
    ps2d_o_b2 = FunMove2d(ps2d_o_b1, ps2d_b1_b2_odo);
    
    this.odo.x = ps2d_o_b2(1);
    this.odo.y = ps2d_o_b2(2);
    this.odo.theta = ps2d_o_b2(3);
    this.odo.lp = lp;
    
    % refresh mark observed
    this.observe;
    
    % save into record file
    this.record;
    
    % draw results
    this.draw;
    
    % print status
    strLp = ['-lp: ', num2str(lp), '; '];
    strPs = ['-pose: ', num2str(ps2d_w_b2(1)), ' ', ...
        num2str(ps2d_w_b2(2)), ' ', num2str(ps2d_w_b2(3)), '; '];
    strVel = ['-vel: ', num2str(this.velLin), ' ', ...
        num2str(this.velRot), '; '];
    strMk = ['-mk: ', num2str(numel(this.mk.id)), '; '];
    disp([strLp, strPs, strVel, strMk]);
    
    % end this loop
    lp = lp + 1;
    pause(0.1);
end

this.stop;

end

