function RenewMeasRec(this, bUseVel)
%RENEWMEASREC Summary of this function goes here
%   Detailed explanation goes here

if nargin == 1
    bUseVel = false;
end

if isempty(this.mkNew)
    return;
end

numMk = numel(this.mkNew.lp);

for i = 1:numMk
    id = this.mkNew.id(i);
    rowRec = find(this.mkRec.id == id);
    
    if isempty(rowRec)
        % add into mkRec
        this.mkRec.lp = [this.mkRec.lp; this.mkNew.lp(i)];
        this.mkRec.id = [this.mkRec.id; this.mkNew.id(i)];
        this.mkRec.rvec = [this.mkRec.rvec; this.mkNew.rvec(i,:)];
        this.mkRec.tvec = [this.mkRec.tvec; this.mkNew.tvec(i,:)];
        this.mkRec.num = numel(this.mkRec.lp);
    else
        % delete old record in odoRec
        lpTmp = this.mkRec.lp(rowRec);
        rowTmp = find(this.mkRec.lp == lpTmp);
        if numel(rowTmp) == 1
            rowTmp2 = find(this.odoRec.lp == lpTmp);
            this.odoRec.lp(rowTmp2) = [];
            this.odoRec.x(rowTmp2) = [];
            this.odoRec.y(rowTmp2) = [];
            this.odoRec.theta(rowTmp2) = [];
            this.odoRec.num = numel(this.odoRec.lp);
            this.odoRec.sigma(rowTmp2,:) = [];
            if bUseVel
                this.odoRec.vx(rowTmp2) = [];
                this.odoRec.vy(rowTmp2) = [];
                this.odoRec.vtheta(rowTmp2) = [];
            end
        end
        
        % delete old record in mkRec
        this.mkRec.lp(rowRec) = [];
        this.mkRec.id(rowRec) = [];
        this.mkRec.rvec(rowRec,:) = [];
        this.mkRec.tvec(rowRec,:) = [];
        
        % add new record in mkRec
        this.mkRec.lp = [this.mkRec.lp; this.mkNew.lp(i)];
        this.mkRec.id = [this.mkRec.id; this.mkNew.id(i)];
        this.mkRec.rvec = [this.mkRec.rvec; this.mkNew.rvec(i,:)];
        this.mkRec.tvec = [this.mkRec.tvec; this.mkNew.tvec(i,:)];
        this.mkRec.num = numel(this.mkRec.lp);
    end
end

% add new record in odoRec
this.odoRec.lp = [this.odoRec.lp; this.odoNew.lp];
this.odoRec.x = [this.odoRec.x; this.odoNew.x];
this.odoRec.y = [this.odoRec.y; this.odoNew.y];
this.odoRec.theta = [this.odoRec.theta; this.odoNew.theta];
this.odoRec.num = numel(this.odoRec.lp);
this.odoRec.sigma{end+1,1} = this.odoNew.sigma;
if bUseVel
    this.odoRec.vx = [this.odoRec.vx; this.odoNew.vx];
    this.odoRec.vy = [this.odoRec.vy; this.odoNew.vy];
    this.odoRec.vtheta = [this.odoRec.vtheta; this.odoNew.vtheta];
end
end

