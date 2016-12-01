function OnKeyPressed( this, hdFig, callBackData )
%ONKEYPRESSED call back function when any key is pressed

key = callBackData.Key;

switch key
    case 'q'
        this.flag.bQuit = true;
    case 'w'
        this.vel_lin = this.vel_lin + 50;
        if this.vel_lin > 1000
            this.vel_lin = 1000;
        end
    case 's'
        this.vel_lin = this.vel_lin - 50;
        if this.vel_lin < -1000
            this.vel_lin = -1000;
        end
    case 'a'
        this.vel_rot = this.vel_rot + pi/50;
        if this.vel_rot > pi/2
            this.vel_rot = pi/2;
        end
    case 'd'
        this.vel_rot = this.vel_rot - pi/50;
        if this.vel_rot < -pi/2
            this.vel_rot = -pi/2;
        end
    case 'e'
        this.vel_rot = 0;
        this.vel_lin = 0;
    otherwise
end

end

