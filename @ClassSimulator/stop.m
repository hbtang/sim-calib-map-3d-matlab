function Stop( this )
%STOP stop simulator process

close(this.hds.hdFigSim);
% fclose(this.OutputFileOdoId);
% fclose(this.OutputFileMkId);

end

