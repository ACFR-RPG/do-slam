function writeDataAssociationObjectIndices(config,nObjects)

GTFileName = config.groundTruthFileName;
filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,GTFileName);

fileID = fopen(filepath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};
fclose(fileID);
IndexC = strfind(CStr, 'DataAssociation');
% find lines with a DataAssociation entry
Index = find(~cellfun('isempty', IndexC));
% count number of vertices
nVertices = 0;
fileID = fopen(filepath,'r');
line = fgetl(fileID);
while ischar(line)
    if strcmp(line(1:length('VERTEX')),'VERTEX')
        nVertices = nVertices+1;
    end
    line = fgetl(fileID);
end
fclose(fileID);

nSE3MotionVertices = 0;
isNewSE3Vertex = 0;
object = 0;
lastObject = 0;
index2Last = 0;
nLandmarksPerMotionVertex = 0;
for j=1:1:length(Index)
    if j > 1
        lastObject = object;
        index2Last = index2;
    end
    % get line of Index
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',Index(j)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    index1 = str2double(splitLine{1,2});
    index2 = str2double(splitLine{1,3});
    object = str2double(splitLine{1,4});
    fclose(fileID);
    if nSE3MotionVertices == 0
        isNewSE3Vertex = 1;
        newVertexID = nVertices +1;
        nLandmarksPerMotionVertex = 2;
    end
    if j>1 && nObjects > 1 && object~=lastObject
        nLandmarksPerMotionVertex = config.newMotionVertexPerNLandmarks;
    end
    
    if j > 1
        if nLandmarksPerMotionVertex < config.newMotionVertexPerNLandmarks
            if index1 ~= index2Last
                nLandmarksPerMotionVertex = nLandmarksPerMotionVertex + 2;
            else
                nLandmarksPerMotionVertex = nLandmarksPerMotionVertex + 1;
            end
        else
        isNewSE3Vertex = 1;
        newVertexID = newVertexID+1;
        nLandmarksPerMotionVertex = 2;
        end
    end
    
    if ~isempty(Index(j))
                label = '2PointsDataAssociation';
                index3 = newVertexID;
        if  isNewSE3Vertex
            nSE3MotionVertices = nSE3MotionVertices +1;
        end
        CStr(end+1)= cellstr(sprintf('%s %d %d %d',...
            label,index1,index2,index3));
        isNewSE3Vertex = 0;
    end
    % Save in the file again
%     fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
%         config.sep,config.graphFileFolderName,config.sep,GTFileName), 'w');
%     fprintf(fileID, '%s\n', CStr{:});
%     fclose(fileID);
    
    % Save in a different file
    filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,GTFileName(1:end-6),'Test.graph');
    fileID = fopen(filepath,'w');
    fprintf(fileID, '%s\n', CStr{:});
    fclose(fileID);
end
newGTFileName = strcat(GTFileName(1:end-6),'Test.graph');
GTFilePath = strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,newGTFileName);
deleteDataAssociationFromGraphFile(GTFilePath)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MeasurementsFileName = config.measurementsFileName;
filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName);

fileID = fopen(filepath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};
fclose(fileID);
IndexC = strfind(CStr, 'DataAssociation');
% find lines with a DataAssociation entry
Index = find(~cellfun('isempty', IndexC));

for j=1:length(Index)
    % get line of Index
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',...
        Index(j)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    index1 = str2double(splitLine{1,2});
    index2 = str2double(splitLine{1,3});
    fclose(fileID);
    
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,newGTFileName),'r');
    Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
    CStrGT = Data{1};
    fclose(fileID);
    searchedStr = strcat({'2PointsDataAssociation'},{' '},{num2str(index1)},...
        {' '},{num2str(index2)},{' '});
    IndexC = strfind(CStrGT, searchedStr);
    edgeIndex = find(~cellfun('isempty', IndexC),1);
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,newGTFileName),'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',edgeIndex-1);
    line = cell2mat(line{1,1});
    splitLine = str2double(strsplit(line,' '));
    index3 = splitLine(1,4);
    fclose(fileID);
    CStr(Index(j)) = cellstr(sprintf('%s %d %d %d',...
        '2PointsDataAssociation',index1,index2,index3));
%     % Save the file again:
%     fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
%         config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName), 'w');
%     fprintf(fileID, '%s\n', CStr{:});
%     fclose(fileID);
    % Save in a different file
    filepath = strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName(1:end-6),'Test.graph');
    fileID = fopen(filepath,'w');
    fprintf(fileID, '%s\n', CStr{:});
    fclose(fileID);
end

end