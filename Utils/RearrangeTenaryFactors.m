function RearrangeTenaryFactors(config)

GTFileName = config.groundTruthFileName;
GTFilepath = strcat(config.folderPath,config.sep,'Data',...
                 config.sep,config.graphFileFolderName,config.sep,GTFileName);
GTFileOutpath = strcat(config.folderPath,config.sep,'Data',...
                 config.sep,config.graphFileFolderName,config.sep,strcat(GTFileName, '_tmp'));
MeaFileName = config.measurementsFileName;
MeaFilepath = strcat(config.folderPath,config.sep,'Data',...
                 config.sep,config.graphFileFolderName,config.sep,MeaFileName);
MeaFileOutpath = strcat(config.folderPath,config.sep,'Data',...
                 config.sep,config.graphFileFolderName,config.sep,strcat(MeaFileName, '_tmp'));

motionObvCount = 0;
motionObvData = {};
motionFormat = strcat('%s', repmat(' %d', [1, 12]));
prevMotionKey = -1;

fileID = fopen(GTFilepath, 'r');
fileOutID = fopen(GTFileOutpath, 'w');
line = fgetl(fileID);
while ischar(line)
    if ~contains(line, 'EDGE_SE3_MOTION')
        fprintf(fileOutID, '%s\n', line);
    else
        scan = textscan(line,motionFormat,'delimiter',' ');
        currMotionKey = cell2mat(scan(4));
        if prevMotionKey == -1
            prevMotionKey = currMotionKey;
        elseif prevMotionKey ~= currMotionKey
            prevMotionKey = currMotionKey;
            if motionObvCount >= 3
                for i = 1:size(motionObvData, 2)
                    fprintf(fileOutID, '%s\n', cell2mat(motionObvData(i)));
                end
            end
            motionObvCount = 0;
            motionObvData = {};
        else
        end
        motionObvData{end+1} = line;
        motionObvCount = motionObvCount + 1;
    end
    line = fgetl(fileID);
end
fclose(fileID);
fclose(fileOutID);

motionObvCount = 0;
motionObvData = {};
motionFormat = strcat('%s', repmat(' %d', [1, 12]));
prevMotionKey = -1;

fileID = fopen(MeaFilepath, 'r');
fileOutID = fopen(MeaFileOutpath, 'w');
line = fgetl(fileID);
while ischar(line)
    if ~contains(line, 'EDGE_SE3_MOTION')
        fprintf(fileOutID, '%s\n', line);
    else
        scan = textscan(line,motionFormat,'delimiter',' ');
        currMotionKey = cell2mat(scan(4));
        if prevMotionKey == -1
            prevMotionKey = currMotionKey;
        elseif prevMotionKey ~= currMotionKey
            prevMotionKey = currMotionKey;
            if motionObvCount >= 3
                for i = 1:size(motionObvData, 2)
                    fprintf(fileOutID, '%s\n', cell2mat(motionObvData(i)));
                end
            end
            motionObvCount = 0;
            motionObvData = {};
        else
        end
        motionObvData{end+1} = line;
        motionObvCount = motionObvCount + 1;
    end
    line = fgetl(fileID);
end
fclose(fileID);
fclose(fileOutID);

fileID = fopen(GTFileOutpath, 'r');
fileOutID = fopen(GTFilepath, 'w');
line = fgetl(fileID);
while ischar(line)
    fprintf(fileOutID, '%s\n', line);
    line = fgetl(fileID);    
end
fclose(fileID);
fclose(fileOutID);
delete(GTFileOutpath);

fileID = fopen(MeaFileOutpath, 'r');
fileOutID = fopen(MeaFilepath, 'w');
line = fgetl(fileID);
while ischar(line)
    fprintf(fileOutID, '%s\n', line);
    line = fgetl(fileID);    
end
fclose(fileID);
fclose(fileOutID);
delete(MeaFileOutpath);

end