function [frameFeatures,globalFeatures] = extractFrameFeatures(K,rgbI,depthI,frameObjects,...
    frame,nFeaturesPerFrame,nFeaturesPerObject,globalFeatures,settings)

features = [];
featuresLocation = [];
featuresObjectId = [];
featuresMoving = [];
featuresOriginFrame = [];
featuresId = [];

% extract features within objects' masks
for i = 1:length(frameObjects)
    objectMask = frameObjects(i).mask;
    
%     objectBoundingBox = frameObjects(i).boundingBox;
%     yCentroid = ceil((objectBoundingBox(2)+objectBoundingBox(4))/2);
%     aboveCentroidObjectMask = [objectMask(1:yCentroid,:);zeros(size(objectMask,1)-yCentroid,size(objectMask,2))];
%     shrankObjectMask = bwmorph(aboveCentroidObjectMask,'thin',6);
    
    shrankObjectMask = bwmorph(objectMask,'thin',6);
    I = rgbI.*repmat(uint8(shrankObjectMask),[1,1,3]);
    corners = detectFASTFeatures(rgb2gray(I));
%     figure;imshow(I);
%     hold on;
%     scatter(corners.Location(:,1),corners.Location(:,2))
    nFeaturesOnCurrentObject = sum([frame.features.objectId]==frameObjects(i).id);
    if nFeaturesPerObject - nFeaturesOnCurrentObject > 0
        strongestFeatures = corners.selectStrongest(nFeaturesPerObject - nFeaturesOnCurrentObject);
        features = mergeFeatureStructs(features, strongestFeatures);
        featuresLocation = [featuresLocation; strongestFeatures.Location];
        nFeatures = length(strongestFeatures);
        % features on objects are assigned the object id as
        % featureObjectId 
        featuresObjectId = [featuresObjectId; repmat(frameObjects(i).id,[length(strongestFeatures),1])];
        featuresMoving = [featuresMoving; repmat(frameObjects(i).moving,[length(strongestFeatures),1])];
        featuresOriginFrame = [featuresOriginFrame; repmat(frame.number,[length(strongestFeatures),1])];
        featuresId = [featuresId; [length(globalFeatures.location3D)+1:length(globalFeatures.location3D) + ...
            length(strongestFeatures)]'];
    end
end

nFeaturesToExtract = nFeaturesPerFrame - length(featuresLocation);
if nFeaturesToExtract > 0
    % features on static background are assigned -1 as featureObjectId
    featuresObjectId = [featuresObjectId;-1*ones(nFeaturesToExtract,1)];
    featuresMoving = [featuresMoving; zeros(nFeaturesToExtract,1)];
    featuresOriginFrame = [featuresOriginFrame; repmat(frame.number,[nFeaturesToExtract,1])];
    rgbICopy = rgbI;
    for i = 1:length(frameObjects)
        objectMask = frameObjects(i).mask;
        thickenedObjectMask = bwmorph(objectMask,'thicken',6);
        I = rgbICopy.*repmat(uint8(~thickenedObjectMask),[1,1,3]);
        rgbICopy = I;
    end
    corners = detectFASTFeatures(rgb2gray(rgbICopy));
    strongestFeatures = corners.selectStrongest(nFeaturesToExtract);
    features = mergeFeatureStructs(features, strongestFeatures);
    featuresLocation = [featuresLocation; strongestFeatures.Location];
    featuresId = [featuresId; [length(globalFeatures.location3D)+1:length(globalFeatures.location3D) + ...
        length(strongestFeatures)]']; 
end

frameFeatures.features = features;
frameFeatures.location = featuresLocation;
frameFeatures.moving   = featuresMoving;
frameFeatures.objectId = featuresObjectId;
frameFeatures.originFrame = featuresOriginFrame;
frameFeatures.id = featuresId;

globalLocation3D = globalFeatures.location3D;
%globalLocation3DNoisyDepth = globalFeatures.location3DNoisyDepth;
globalWeight = globalFeatures.weight;
globalId = globalFeatures.id;
globalFrame = globalFeatures.frame;
globalCameraLocation = globalFeatures.cameraLocation;
globalStatic = globalFeatures.static;
globalObjectId = globalFeatures.objectId;
globalAssociation = globalFeatures.dynamicAssociation;

featuresLocation3D = zeros(3,length(frameFeatures.location));
%featuresLocation3DNoisyDepth = zeros(3,length(frameFeatures.location));
for i=1:size(frameFeatures.location,1)
    pixelRow = frameFeatures.location(i,2);
    pixelCol = frameFeatures.location(i,1);
    if strcmp(settings.depth,'SPSS')
        %kitti
        pixelDisparity = double(depthI(pixelRow,pixelCol))/256;
        if pixelDisparity == 0
            continue;
        else
            gtPixelDepth = K(1,1)*0.537/pixelDisparity;
        end
    elseif strcmp(settings.depth,'GT')
        % vkitti
        gtPixelDepth = double(depthI(pixelRow,pixelCol))/100;
        % apply depth noise -- if vkitti testing noisy depth
         if settings.applyDepthNoise
            depthNoiseSigma = (gtPixelDepth^2*0.15)/(settings.K(1,1)*0.54); % sigma = z^2*delta_d/f*b
            pixelDepthNoise = normrnd(0,depthNoiseSigma,[1 1]);
            pixelDepth = gtPixelDepth + pixelDepthNoise;
         end
    end
    % image--> camera
    camera3DPoint = K\[pixelCol;pixelRow;1];
    camera3DPoint = camera3DPoint * gtPixelDepth;
    %camera3DPointNoisyDepth = camera3DPoint * pixelDepth;
    % camera --> world
    cameraPoseMatrix = poseToTransformationMatrix(frame.cameraPose);
    world3DPoint = cameraPoseMatrix * [camera3DPoint;1];
    %world3DPointNoisyDepth = cameraPoseMatrix * [camera3DPointNoisyDepth;1];
    featuresLocation3D(:,i) = world3DPoint(1:3);
    %featuresLocation3DNoisyDepth(:,i) = world3DPointNoisyDepth(1:3);
    % global features
    if frameFeatures.moving(i) == 0
        if isempty(globalLocation3D)
            globalLocation3D(:,1) = world3DPoint(1:3,1);
            %globalLocation3DNoisyDepth(:,1) = world3DPointNoisyDepth(1:3,1);
            globalWeight(1,1) = 1;
            globalId(1,1) = frameFeatures.id(i);
            globalFrame(1,1) = frameFeatures.originFrame(i);
            globalCameraLocation(:,1) = camera3DPoint;
            globalStatic(1,1) = 1;
            globalObjectId(1,1) = -1;
        else
            % compute distance to all 3D points
            distances = sqrt(bsxfun(@plus,bsxfun(@plus,...
                (globalLocation3D(1,:).'-world3DPoint(1,1)).^2,...
                (globalLocation3D(2,:).'-world3DPoint(2,1)).^2),...
                (globalLocation3D(3,:).'-world3DPoint(3,1)).^2))';
            % find min distant point
            [~,index] = find(distances == min(distances));
            % if closest point is within distance threshold in 3D
            if(norm(world3DPoint(1:3,1) - globalLocation3D(:,index)) <= settings.distanceThreshold)
                globalWeight(index,1) = globalWeight(index,1) + 1;
            else
                globalLocation3D = [globalLocation3D, world3DPoint(1:3)];
                %globalLocation3DNoisyDepth = [globalLocation3DNoisyDepth, world3DPointNoisyDepth(1:3)];
                globalWeight(end+1,1) = 1;
                globalId(end+1,1) = size(globalLocation3D,2);
                globalFrame(end+1,1) = frameFeatures.originFrame(i);
                globalCameraLocation = [globalCameraLocation, camera3DPoint(1:3)];
                globalStatic(end+1,1) = 1;
                globalObjectId(end+1,1) = -1;
            end
        end
    else
        % dynamic point
        if isempty(globalLocation3D)
            globalLocation3D(:,1) = world3DPoint(1:3,1);
            %globalLocation3DNoisyDepth(:,1) = world3DPointNoisyDepth(1:3,1);
            globalWeight(1,1) = 1;
            globalId(1,1) = frameFeatures.id(i);
            globalFrame(1,1) = frameFeatures.originFrame(i);
            globalCameraLocation(:,1) = camera3DPoint;
            globalStatic(1,1) = 0;
            globalObjectId(1,1) = frameFeatures.objectId(i);
        else
            % compute distance to all 3D points
            distances = sqrt(bsxfun(@plus,bsxfun(@plus,...
                (globalLocation3D(1,:).'-world3DPoint(1,1)).^2,...
                (globalLocation3D(2,:).'-world3DPoint(2,1)).^2),...
                (globalLocation3D(3,:).'-world3DPoint(3,1)).^2))';
            % find min distant point
            [~,index] = find(distances == min(distances));
            % if closest point is within distance threshold in 3D
            if(norm(world3DPoint(1:3,1) - globalLocation3D(:,index)) <= settings.distanceThreshold)
                for m = 1:length(index)
                    % if point has changed object or is no longer moving, skip 
                    if globalStatic(index(m),1) ~= 0 || globalObjectId(index(m),1) ~= frameFeatures.objectId(i)
                        disp('A point has changed object id or is no longer dynamic')
                        continue;
                    end 
                end
            else
                globalLocation3D = [globalLocation3D, world3DPoint(1:3)];
                %globalLocation3DNoisyDepth = [globalLocation3DNoisyDepth, world3DPointNoisyDepth(1:3)];
                globalWeight(end+1,1) = 1;
                globalId(end+1,1) = size(globalLocation3D,2);
                globalFrame(end+1,1) = frameFeatures.originFrame(i);
                globalCameraLocation = [globalCameraLocation, camera3DPoint(1:3)];
                globalStatic(end+1,1) = 0;
                globalObjectId(end+1,1) = frameFeatures.objectId(i);
            end
        end
        % globalAssociation: each row contains a unique object id and its tracklets of points
        if isempty(globalAssociation)
            globalAssociation{1,1} = frameFeatures.objectId(i);
            globalAssociation{1,2} = size(globalLocation3D,2);
        else
            % new dynamic 3D point - start new tracklet
            if(norm(world3DPoint(1:3,1) - globalLocation3D(:,index)) > settings.distanceThreshold+0.001)
                objectIds = [globalAssociation{:,1}];
                idx = find(objectIds == frameFeatures.objectId(i));
                % new tracklet on an existing object
                if ~isempty(idx)
                    l = size(globalAssociation(idx,:),2);
                    globalAssociation{idx,l+1} = size(globalLocation3D,2);
                % new tracklet on a new object
                else
                    l = size(globalAssociation,1);
                    globalAssociation{l+1,1} = frameFeatures.objectId(i);
                    globalAssociation{l+1,2} = size(globalLocation3D,2);
                end
            end
        end 
    end
end
frameFeatures.location3D = featuresLocation3D;
%frameFeatures.location3DNoisyDepth = featuresLocation3DNoisyDepth;

globalFeatures.location3D = globalLocation3D;
%globalFeatures.location3DNoisyDepth = globalLocation3DNoisyDepth;
globalFeatures.weight = globalWeight;
globalFeatures.id = globalId;
globalFeatures.frame = globalFrame;
globalFeatures.cameraLocation = globalCameraLocation;
globalFeatures.static = globalStatic;
globalFeatures.objectId = globalObjectId;
globalFeatures.dynamicAssociation = globalAssociation;

end