function onKeyPressed( this, hdFig, callBackData )
%ONKEYPRESSED call back function when any key is pressed

key = callBackData.Key;

switch key
    case 'q'
        this.ifQuit = true;
    case 'w'
        this.velLin = this.velLin + 50;
        if this.velLin > 1000
            this.velLin = 1000;
        end
    case 's'
        this.velLin = this.velLin - 50;
        if this.velLin < -1000
            this.velLin = -1000;
        end
    case 'a'
        this.velRot = this.velRot + pi/50;
        if this.velRot > pi/2
            this.velRot = pi/2;
        end
    case 'd'
        this.velRot = this.velRot - pi/50;
        if this.velRot < -pi/2
            this.velRot = -pi/2;
        end
    case 'e'
        this.velRot = 0;
        this.velLin = 0;
    otherwise
end

end

