function [ configXml ] = readXml_vmckf( fileConfigXml )
%READ_XML Summary of this function goes here
%   Detailed explanation goes here

xmlDoc = xmlread(fileConfigXml);

configStrNameArray = {'PathFold', 'NameMk', 'NameOdo'};
configDataNameArray = {'MuXTrue', ...
    'ThreshTransPruneData', 'ThreshRotPruneData',...
    'rcbInit', 'rcbSigmaInit', 'pt3cbInit', 'pt3cbSigmaInit',...
    'SolverConfig_StdErrRatioOdoLin', 'SolverConfig_StdErrRatioOdoRot', ...
    'SolverConfig_MinStdErrOdoLin', 'SolverConfig_MinStdErrOdoRot', ...
    'SolverConfig_StdErrRatioMkX', 'SolverConfig_StdErrRatioMkY', 'SolverConfig_StdErrRatioMkZ'};
configXml = struct;

for i = 1:numel(configStrNameArray)    
    nameElement = configStrNameArray{i};    
    arrayElement = xmlDoc.getElementsByTagName(nameElement);
    if arrayElement.getLength == 1
        dataElement = char(arrayElement.item(0).getFirstChild.getData);
    else
        error('Error in reading xml file.');
    end    
    configXml = setfield(configXml, nameElement, dataElement);
end

for i = 1:numel(configDataNameArray)    
    nameElement = configDataNameArray{i};    
    arrayElement = xmlDoc.getElementsByTagName(nameElement);
    if arrayElement.getLength == 1
        dataElement = char(arrayElement.item(0).getFirstChild.getData);
        dataElement = str2num(dataElement);
    else
        error('Error in reading xml file.');
    end    
    configXml = setfield(configXml, nameElement, dataElement);
end

end

