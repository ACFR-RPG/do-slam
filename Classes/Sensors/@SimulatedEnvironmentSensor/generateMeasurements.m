%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17, Yash Vyas -
% yjvyas@gmail.com - 30/05/2017
% Contributors:
%--------------------------------------------------------------------------

function generateMeasurements(self,config)
%GENERATEMEASUREMENTSOCCLUSION very similar to generateMeasurements for the
%SimulatedEnvironmentSensor class, but includes extraction of mesh points
%for use.
%% 1. Initialise variables
% load frequently accessed variables from config
graphFileFolderPath = strcat(config.folderPath,config.sep,'Data',config.sep,config.graphFileFolderName);
if ~exist(graphFileFolderPath,'dir')
    mkdir(graphFileFolderPath)
end
gtFileID = fopen(strcat(config.folderPath,config.sep,'Data',...
                 config.sep,config.graphFileFolderName,config.sep,config.groundTruthFileName),'w');
mFileID  = fopen(strcat(config.folderPath,config.sep,'Data',...
                 config.sep,config.graphFileFolderName,config.sep,config.measurementsFileName),'w');
t      = config.t;
nSteps = numel(t);

% indexing variables
vertexCount         = 0;
objectCount         = 0;
cameraVertexIndexes = zeros(1,nSteps);
pointVertexIndexes = zeros(1,nSteps);
objectVertexIndexes = zeros(self.nObjects, nSteps);
motionVertexIndices = zeros(self.nObjects, nSteps);

% find indexes for static and dynamic points
staticPointLogical      = self.get('points').get('static');
if ~isempty(self.get('objects'))
    staticObjectLogical  = self.get('objects').get('static');
    dynamicPointLogical  = ~staticPointLogical;
    dynamicObjectLogical = ~staticObjectLogical;
else
    staticObjectLogical  = [];
    dynamicPointLogical  = [];
    dynamicObjectLogical = [];
end
staticPointIndexes      = find(staticPointLogical);
staticObjectIndexes  = find(staticObjectLogical);
dynamicPointIndexes     = find(dynamicPointLogical);
dynamicObjectIndexes = find(dynamicObjectLogical);

currObjPosesNoisy = GP_Pose.empty;
prevObjPosesNoisy = GP_Pose.empty;

% error check for visibility
if isempty(self.pointVisibility)
    error('Visibility must be set first.');
end

%% 1.a Clear vertex and object Indexes, instantiate object indexes
for j=1:self.nPoints
    self.get('points',j).clearIndex();
end

for j=1:self.nObjects
    self.get('objects',j).set('vertexIndex',j) % sets the index of the object to its default
    objectCount = j;
end

%% 2. Loop over timestep, simulate observations, write to graph file
for i = 1:nSteps
    %sensor @ time t
    currentSensorPose = self.get('GP_Pose',t(i));
    % Perturb the current pose
    currentSensorPoseNoisy = currentSensorPose.addNoise(config.noiseModel,zeros(size(config.stdPosePose)),config.stdPosePose);
    switch config.poseParameterisation
        case 'R3xso3'
            value = currentSensorPose.get('R3xso3Pose');
            valueMeas = currentSensorPoseNoisy.get('R3xso3Pose');
            quat = rot2quat(value(4:6));
            value = [value(1:3); quat];
            quatMeas = rot2quat(valueMeas(4:6));
            valueMeas = [valueMeas(1:3); quatMeas];
            if i > 1
            prevValue = self.get('GP_Pose',t(i-1)).get('R3xso3Pose');
            prevQuat = rot2quat(prevValue(4:6));
            prevValue = [prevValue(1:3); prevQuat];
            end
        case 'logSE3'
            value = currentSensorPose.get('logSE3Pose');
            if i > 1 
            prevValue = self.get('GP_Pose',t(i-1)).get('logSE3Pose');
            valueMeas = currentSensorPoseNoisy.get('logSE3Pose');
            end
        otherwise
            error('Error: unsupported pose parameterisation')
    end
    if i == 1 
        vertexCount = vertexCount + 1;
        cameraVertexIndexes(i) = vertexCount;
        %WRITE VERTEX TO FILE
        label = config.poseVertexLabel;
        index = cameraVertexIndexes(i);
        writeVertex(label,index,value,gtFileID);
        writeVertex(label,index,value,mFileID);
    elseif (i > 1) && ~prod(value == prevValue)
        vertexCount = vertexCount + 1;
        cameraVertexIndexes(i) = vertexCount;
        %WRITE VERTEX TO FILE
        label = config.poseVertexLabel;
        index = cameraVertexIndexes(i);
        writeVertex(label,index,value,gtFileID);
        writeVertex(label,index,valueMeas,mFileID);
    elseif (i > 1) && any(value == prevValue)
        cameraVertexIndexes(i) = cameraVertexIndexes(i-1);
    end

    %odometry
    if i> 1
        prevSensorPose = self.get('GP_Pose',t(i-1));
        poseRelative = currentSensorPose.AbsoluteToRelativePose(prevSensorPose);
        poseRelativeNoisy = currentSensorPoseNoisy.AbsoluteToRelativePose(prevSensorPoseNoisy);
        %WRITE EDGE TO FILE
        label = config.posePoseEdgeLabel;
        switch config.poseParameterisation
            case 'R3xso3'
                valueGT   = poseRelative.get('R3xso3Pose');
                valueMeas = poseRelativeNoisy.get('R3xso3Pose');
                quat = rot2quat(valueGT(4:6));
                valueGT = [valueGT(1:3); quat];
                quatMeas = rot2quat(valueMeas(4:6));
                valueMeas = [valueMeas(1:3); quatMeas];
            case 'logSE3'
                valueGT   = poseRelative.get('logSE3Pose');
                valueMeas = poseRelativeNoisy.get('logSE3Pose');
            otherwise
                error('Error: unsupported pose parameterisation')
        end
        if ~prod(valueGT == zeros(length(valueGT),1))
            index1 = cameraVertexIndexes(i-1);
            index2 = cameraVertexIndexes(i);
            covariance = config.covPosePose;
            writeEdge(label,index1,index2,valueGT,covariance,gtFileID);
            writeEdge(label,index1,index2,valueMeas,covariance,mFileID);
        end
    else
        label = 'EDGE_SE3_PRIOR';
        index1 = 1;
        value = [0; 0; 0; 0; 0; 0; 1];
        covariance = config.covPosePose;
        writeEdgePrior(label,index1,value,covariance,gtFileID);
        writeEdgePrior(label,index1,value,covariance,mFileID);
    end
    % Store the current pose noisy into prev
    prevSensorPoseNoisy = currentSensorPoseNoisy; 


    % Dynamic object poses
    for j = 1:objectCount
        jObject = self.get('objects',j);
        currObjPose = jObject.get('trajectory').get('GP_Pose',t(i));
        currObjPoseNoisy = currObjPose.addNoise(config.noiseModel,zeros(size(config.stdPosePose)),config.stdPosePose);
        currObjPosesNoisy(j) = currObjPoseNoisy;
        switch config.poseParameterisation
            case 'R3xso3'
                value = currObjPose.get('R3xso3Pose');
                valueMeas = currObjPoseNoisy.get('R3xso3Pose');
                quat = rot2quat(value(4:6));
                value = [value(1:3); quat];
                quatMeas = rot2quat(valueMeas(4:6));
                valueMeas = [valueMeas(1:3); quatMeas];
                if i > 1
                prevValue = self.get('GP_Pose',t(i-1)).get('R3xso3Pose');
                prevQuat = rot2quat(prevValue(4:6));
                prevValue = [prevValue(1:3); prevQuat];
                end
            case 'logSE3'
                value = currObjPose.get('logSE3Pose');
                if i > 1 
                prevValue = self.get('GP_Pose',t(i-1)).get('logSE3Pose');
                valueMeas = currObjPoseNoisy.get('logSE3Pose');
                end
            otherwise
                error('Error: unsupported pose parameterisation')
        end
        vertexCount = vertexCount + 1;
        objectVertexIndexes(j, i) = vertexCount;
        %WRITE VERTEX TO FILE
        label = config.poseVertexLabel;
        index = objectVertexIndexes(j, i);
        writeVertex(label,index,value,gtFileID);
        writeVertex(label,index,valueMeas,mFileID);
    end

    % Dynamic object motions
    if i > 1
        for j = 1:objectCount
            jObject = self.get('objects',j);
            currObjPose = jObject.get('trajectory').get('GP_Pose',t(i));
            PrevObjPose = jObject.get('trajectory').get('GP_Pose',t(i-1));
            currObjPoseNoisy = currObjPosesNoisy(j);
            prevObjPoseNoisy = prevObjPosesNoisy(j);
            objMotion = currObjPose.AbsoluteToRelativePose(PrevObjPose);
            objMotionGlobal = GP_Pose(PrevObjPose.RelativePoseGlobalFrameSE3(objMotion), 'SE3');
            objMotionNoisy = currObjPoseNoisy.AbsoluteToRelativePose(prevObjPoseNoisy);
            objMotionGlobalNoisy = GP_Pose(prevObjPoseNoisy.RelativePoseGlobalFrameSE3(objMotionNoisy), 'SE3');
            switch config.poseParameterisation
                case 'R3xso3'
                    valueGT   = objMotionGlobal.get('R3xso3Pose');
                    valueMeas = objMotionGlobalNoisy.get('R3xso3Pose');
                    quat = rot2quat(valueGT(4:6));
                    valueGT = [valueGT(1:3); quat];
                    quatMeas = rot2quat(valueMeas(4:6));
                    valueMeas = [valueMeas(1:3); quatMeas];
                case 'logSE3'
                    valueGT   = objMotionGlobal.get('logSE3Pose');
                    valueMeas = objMotionGlobalNoisy.get('logSE3Pose');
                otherwise
                    error('Error: unsupported pose parameterisation')
            end
            vertexCount = vertexCount + 1;
            motionVertexIndices(j, i) = vertexCount;
            %WRITE VERTEX TO FILE
            label = config.poseVertexLabel;
            index = motionVertexIndices(j, i);
            writeVertex(label,index,valueGT,gtFileID);
            writeVertex(label,index,valueMeas,mFileID);
        end
    end

    % point observations
    for j = staticPointIndexes
        jPoint = self.get('points',j);
        jPointVisible = self.pointVisibility(j,i);
        jPointRelative = self.pointObservationRelative(j,i);
        if jPointVisible
            %check if point observed before
            if isempty(jPoint.get('vertexIndex'))
                vertexCount = vertexCount + 1;
                jPoint.set('vertexIndex',vertexCount); %*Passed by reference - changes point
                %WRITE VERTEX TO FILE
                label = config.pointVertexLabel;
                index = jPoint.get('vertexIndex');
                value = jPoint.get('R3Position',t(i));
                writeVertex(label,index,value,gtFileID);
%                 valueMeas = jPointNoisy.get('R3Position',t(i));
                writeVertex(label,index,value,mFileID); % DerSpiritus - Still GT points
            elseif (i>1) && (~self.pointVisibility(j,i-1)) && strcmp(config.staticDataAssociation,'Off' )
                label = config.pointVertexLabel;
                vertexCount = vertexCount + 1;
                vertexIndex = [jPoint.get('vertexIndex') vertexCount];
                jPoint.set('vertexIndex',vertexIndex);
                index = vertexIndex(end);
                value = jPoint.get('R3Position',t(i));
                writeVertex(label,index,value,gtFileID);
%                 valueMeas = jPointNoisy.get('R3Position',t(i));
                writeVertex(label,index,value,mFileID); % DerSpiritus - Still GT points
            end
            
            %WRITE EDGE TO FILE
            label = config.posePointEdgeLabel;
            switch config.poseParameterisation
                case 'R3xso3'
                    jPointRelativeNoisy = jPointRelative.addNoise(config.noiseModel,zeros(size(config.stdPosePoint)),config.stdPosePoint); 
                    valueGT   = jPointRelative.get('R3Position');
                    valueMeas = jPointRelativeNoisy.get('R3Position');
                case 'logSE3'
                    jPointRelativeLogSE3      = jPoint.get('GP_Point',t(i)).AbsoluteToRelativePoint(self.get('GP_Pose',t(i)),'logSE3');
                    jPointRelativeLogSE3Noisy = jPointRelativeLogSE3.addNoise(config.noiseModel,zeros(size(config.stdPosePoint)),config.stdPosePoint); 
                    valueGT   = jPointRelativeLogSE3.get('R3Position');
                    valueMeas = jPointRelativeLogSE3Noisy.get('R3Position');
                otherwise
                    error('Error: unsupported pose parameterisation')
            end
            covariance = config.covPosePoint;
            indexCam = cameraVertexIndexes(i);
            vertexIndex = jPoint.get('vertexIndex');
            indexPoint = vertexIndex(end); 
            writeEdge(label,indexCam,indexPoint,valueGT,covariance,gtFileID);
            writeEdge(label,indexCam,indexPoint,valueMeas,covariance,mFileID);
        end
    end
    
    % Dynamic point observations
    for j=dynamicPointIndexes
        jPoint = self.get('points',j);
        jPointVisible = self.pointVisibility(j,i);
        jPointRelative = self.pointObservationRelative(j,i);
        
        % only creates new index if visible
        if jPointVisible
        % add new vertex index to existing ones
            vertexCount = vertexCount + 1;
            vertexIndexes = [jPoint.get('vertexIndex') vertexCount];
            jPoint.set('vertexIndex',vertexIndexes); % sets new vertex
            
            label = config.pointVertexLabel;
            index = vertexIndexes(end);
            value = jPoint.get('R3Position',t(i));
            writeVertex(label,index,value,gtFileID);
            writeVertex(label,index,value,mFileID);% DerSpiritus - Still GT points
            
            %WRITE SENSOR OBSERVATION EDGE TO FILE
            label = config.posePointEdgeLabel;
            switch config.poseParameterisation
                case 'R3xso3'
                    jPointRelativeNoisy = jPointRelative.addNoise(config.noiseModel,zeros(size(config.stdPosePoint)),config.stdPosePoint); 
                    valueGT   = jPointRelative.get('R3Position');
                    valueMeas = jPointRelativeNoisy.get('R3Position');
                case 'logSE3'
                    jPointRelativeLogSE3      = jPoint.get('GP_Point',t(i)).AbsoluteToRelativePoint(self.get('GP_Pose',t(i)),'logSE3');
                    jPointRelativeLogSE3Noisy = jPointRelativeLogSE3.addNoise(config.noiseModel,zeros(size(config.stdPosePoint)),config.stdPosePoint); 
                    valueGT   = jPointRelativeLogSE3.get('R3Position');
                    valueMeas = jPointRelativeLogSE3Noisy.get('R3Position');
                otherwise
                    error('Error: unsupported pose parameterisation')
            end
            covariance = config.covPosePoint;
            indexCam = cameraVertexIndexes(i);
            indexPoint = vertexIndexes(end);
            writeEdge(label,indexCam,indexPoint,valueGT,covariance,gtFileID);
            writeEdge(label,indexCam,indexPoint,valueMeas,covariance,mFileID);
            
            % Associated currently observed points with previously seen
            % ones
            if (i > 1) && (self.pointVisibility(j,i-1))
                % write edge between points if point was visible in
                % previous step
                switch config.pointMotionMeasurement
                    case 'point2DataAssociation'
                        label = config.pointsDataAssociationLabel;
                        indexPointPrev = vertexIndexes(end-1);
                        indexPointCurr = vertexIndexes(end);
                        object = jPoint.get('objectIndexes');
                        objectIndex = self.get('objects',object(1)).get('vertexIndex');
                        valueGT = zeros(3, 1);
                        valueMeas = zeros(3, 1);
%                         % Get connected object motion
%                         jObject = self.get('objects', objectIndex(end));
%                         currObjPose = jObject.get('trajectory').get('GP_Pose',t(i));
%                         PrevObjPose = jObject.get('trajectory').get('GP_Pose',t(i-1));
%                         currObjPoseNoisy = currObjPosesNoisy(objectIndex(end));
%                         prevObjPoseNoisy = prevObjPosesNoisy(objectIndex(end));
%                         objMotion = currObjPose.AbsoluteToRelativePose(PrevObjPose);
%                         objMotionNoisy = currObjPoseNoisy.AbsoluteToRelativePose(prevObjPoseNoisy);
%                         switch config.poseParameterisation
%                             case 'R3xso3'
%                                 valueGT   = objMotion.get('R3xso3Pose');
%                                 valueMeas = objMotionNoisy.get('R3xso3Pose');
%                                 quat = rot2quat(valueGT(4:6));
%                                 valueGT = [valueGT(1:3); quat];
%                                 quatMeas = rot2quat(valueMeas(4:6));
%                                 valueMeas = [valueMeas(1:3); quatMeas];
%                             case 'logSE3'
%                                 valueGT   = objMotion.get('logSE3Pose');
%                                 valueMeas = objMotionNoisy.get('logSE3Pose');
%                             otherwise
%                                 error('Error: unsupported pose parameterisation')
%                         end
                        covariance = config.covPosePoint;
                        covariance = covToUpperTriVec(covariance);
                        cov_inv = covariance;
                        cov_inv(cov_inv~=0) = 1./cov_inv(cov_inv~=0);
                        fprintf(gtFileID,'%s %d %d %d',label,indexPointPrev,indexPointCurr,motionVertexIndices(objectIndex(end), i));
                        formatSpec = strcat(repmat(' %0.9f',1,numel(valueGT)), repmat(' %0.9f',1,numel(cov_inv)),'\n');
                        fprintf(gtFileID,formatSpec,valueGT,cov_inv);
                        fprintf(mFileID,'%s %d %d %d',label,indexPointPrev,indexPointCurr,motionVertexIndices(objectIndex(end), i));
                        formatSpec = strcat(repmat(' %0.9f',1,numel(valueGT)), repmat(' %0.9f',1,numel(cov_inv)),'\n');
                        fprintf(mFileID,formatSpec,valueMeas,cov_inv);
                    case 'point2Edge'
                        label = config.pointPointEdgeLabel;
                        covariance = config.covPointPoint;
                        index1 = vertexIndexes(end);
                        index2 = vertexIndexes(end-1);
                        valueGT = jPoint.get('R3Position',t(i))-jPoint.get('R3Position',t(i-1));
                        pointMotion = GP_Point(valueGT);
                        pointMotionNoisy = pointMotion.addNoise(config.noiseModel,zeros(size(config.stdPointPoint)),config.stdPointPoint); 
                        valueMeas = pointMotionNoisy.get('R3Position');
                        writeEdge(label,index1,index2,valueGT,covariance,gtFileID);
                        writeEdge(label,index1,index2,valueMeas,covariance,mFileID);
                    case 'point3Edge'
                        if (i > 2) && (self.pointVisibility(j,i-2)) % checks for second condition - whether last two were observed
                            index1 = vertexIndexes(end);
                            index2 = vertexIndexes(end-1);
                            index3 = vertexIndexes(end-2);
                            label = config.point3EdgeLabel;
                            if strcmp(config.motionModel,'constantSpeed')
                                valueGT = norm(jPoint.get('R3Position',t(i)) - jPoint.get('R3Position',t(i-1)))-...
                                          norm(jPoint.get('R3Position',t(i-1)) - jPoint.get('R3Position',t(i-2)));
                            elseif strcmp(config.motionModel,'constantVelocity')
                                valueGT = (jPoint.get('R3Position',t(i)) - jPoint.get('R3Position',t(i-1)))-...
                                          (jPoint.get('R3Position',t(i-1)) - jPoint.get('R3Position',t(i-2)));
                            end
                            covariance = config.cov3Points;
                            point3Motion = GP_Point(valueGT);
                            point3MotionNoisy = point3Motion.addNoise(config.noiseModel,zeros(size(config.std3Points)),config.std3Points);
                            valueMeas = point3MotionNoisy.get('R3Position');
                            writeEdge(label,[index1 index2],index3,valueGT,covariance,gtFileID);
                            writeEdge(label,[index1 index2],index3,valueMeas,covariance,mFileID);
                        end
                    case 'velocity'
                        if (i > 2) && (self.pointVisibility(j,i-2)) % checks for second condition
                            % write velocity vertex and set indexes for
                            % edges
                            vertexCount = vertexCount + 1;
                            
                            % look at constraint motion Model and implement
                            % vertex and edge
                            if strcmp(config.motionModel,'constantSpeed')
                                value = mean([norm(jPoint.get('R3Position',t(i))-jPoint.get('R3Position',t(i-1))),...
                                              norm(jPoint.get('R3Position',t(i-1))-jPoint.get('R3Position',t(i-2)))]);
                                valueGT1_2 = value-norm(jPoint.get('R3Position',t(i-1))-jPoint.get('R3Position',t(i-2)));
                                valueGT2_3 = value-norm(jPoint.get('R3Position',t(i))-jPoint.get('R3Position',t(i-1)));
                            elseif strcmp(config.motionModel,'constantVelocity')
                                value = mean([jPoint.get('R3Position',t(i))-jPoint.get('R3Position',t(i-1)),...
                                              jPoint.get('R3Position',t(i-1))-jPoint.get('R3Position',t(i-2))]);
                                valueGT1_2 = value-(jPoint.get('R3Position',t(i-1))-jPoint.get('R3Position',t(i-2)));
                                valueGT2_3 = value-(jPoint.get('R3Position',t(i))-jPoint.get('R3Position',t(i-1)));
                            end
                            writeVertex(config.velocityVertexLabel,vertexCount,value,gtFileID);
                            % write velocity edges
                            % point @ time 1,2 - velocity    
                            edgeLabel = config.pointVelocityEdgeLabel;
                            index1 = vertexIndexes(end-2);
                            index2 = vertexIndexes(end-1);
                            index3 = vertexCount;
                            velocityEdge1_2 = GP_Point(valueGT1_2).addNoise(config.noiseModel,zeros(size(config.std2PointsVelocity)),config.std2PointsVelocity);
                            valueMeas1_2 = velocityEdge1_2.get('R3Position');
                            velocityEdge2_3 = GP_Point(valueGT2_3).addNoise(config.noiseModel,zeros(size(config.std2PointsVelocity)),config.std2PointsVelocity);
                            valueMeas2_3 = velocityEdge2_3.get('R3Position');
                            writeEdge(edgeLabel,[index1 index2],index3,valueGT1_2,covariancegtFileID);
                            writeEdge(edgeLabel,[index1 index2],index3,valueMeas1_2,covariance,mFileID);
                            
                            % point @ time 2,3 - velocity
                            index1 = vertexIndexes(end-1);
                            index2 = vertexIndexes(end);
                            index3 = vertexCount;
                            writeEdge(edgeLabel,[index1 index2],index3,valueGT2_3,covariance,gtFileID);
                            writeEdge(edgeLabel,[index1 index2],index3,valueMeas2_3,covariance,mFileID);
                        end
                    case 'Off'
                    otherwise
                        error('Point motion measurement type is unidentified.')
                end
            end


        end
    end
    prevObjPosesNoisy = currObjPosesNoisy;

    
    %point-plane observations
    for j = staticObjectIndexes
        jObject = self.get('objects',j);
        jPointIndexes = jObject.get('pointIndexes');
        jPointVisibility = logical(self.pointVisibility(jPointIndexes,i));
        jNVisiblePoints  = sum(jPointVisibility);
        jVisiblePointIndexes = jPointIndexes(jPointVisibility);
        %check visibility
        if jNVisiblePoints > 3
            %plane visible
            self.objectVisibility(jObject.get('index'),i) = 1;
            
            if isempty(jObject.get('vertexIndex'))
                vertexCount = vertexCount + 1;
                jObject.set('vertexIndex',vertexCount); %*Passed by reference - changes object
                %WRITE VERTEX TO FILE
                label = config.planeVertexLabel;
                index = jObject.get('vertexIndex');
                value = jObject.get('parameters',t(i));
                writeVertex(label,index,value,gtFileID);
            end
        end
        %object observed previously, create point-plane edges
        if ~isempty(jObject.get('vertexIndex'))
            for k = 1:jNVisiblePoints
                %WRITE EDGE TO FILE
                label = config.pointPlaneEdgeLabel;
                valueGT = 0;
                valueMeas = valueGT;
                covariance = config.covPointPlane;
                index1 = self.get('points',jVisiblePointIndexes(k)).get('vertexIndex');
                index2 = jObject.get('vertexIndex');
                writeEdge(label,index1,index2,valueGT,covariance,gtFileID);
                writeEdge(label,index1,index2,valueMeas,covariance,mFileID);
            end
        end
    end
    
    for j = 1:self.nObjects
        if i > 1
            if self.objectVisibility(j,i) && ~self.objectVisibility(j,i-1) && strcmp(config.objectAssociation,'Off')
                objectCount = objectCount + 1; 
                objectIndex = [self.get('objects',j).get('vertexIndex') objectCount];
                self.get('objects',j).set('vertexIndex',objectIndex);
            end
        end
    end
end

fclose(gtFileID);
fclose(mFileID);

end

