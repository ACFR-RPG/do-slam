function [images, wrongAssociations] = objectTracker(images)

wrongAssociations = 0;
maxID = max([images(1).objects.id]);

for i=2:length(images)
    
    currentImage = images(i);
    previousImage = images(i-1);
    nObjectCurrentImage  = currentImage.nObjects;
    nObjectPreviousImage = previousImage.nObjects;
    
    % leftImageBoundary = [0,0;imageSize(1),0];
    % rightImageBoundary = [0,imageSize(2);imageSize(1),imageSize(2)];
    % topImageBoundary = [0,0;0,imageSize(2)];
    % bottomImageBoundary = [imageSize(1),0;imageSize(1),imageSize(2)];
    
    for j=1:nObjectCurrentImage
        %get Object Centroid
        objectCentroid = currentImage.objects(j).centroid;
        
        %     %determine possibility of object entering or leaving image
        %         %determine distance to left boundary
        %         distanceCentroidLeftImageBoundary = norm(cross(leftImageBoundary(1,:)-...
        %             leftImageBoundary(2,:),objectCentroid-leftImageBoundary(2,:))) / ...
        %             norm(leftImageBoundary(1,:)-leftImageBoundary(2,:));
        %         %determine distance to right boundary
        %         distanceCentroidRightImageBoundary = norm(cross(rightImageBoundary(1,:)-...
        %             rightImageBoundary(2,:),objectCentroid-rightImageBoundary(2,:))) / ...
        %             norm(rightImageBoundary(1,:)-rightImageBoundary(2,:));
        %         %determine distance to top boundary
        %         distanceCentroidTopImageBoundary = norm(cross(topImageBoundary(1,:)-...
        %             topImageBoundary(2,:),objectCentroid-topImageBoundary(2,:))) / ...
        %             norm(topImageBoundary(1,:)-topImageBoundary(2,:));
        %         %determine distance to bottom boundary
        %         distanceCentroidBottomImageBoundary = norm(cross(bottomImageBoundary(1,:)-...
        %             bottomImageBoundary(2,:),objectCentroid-bottomImageBoundary(2,:))) / ...
        %             norm(bottomImageBoundary(1,:)-bottomImageBoundary(2,:));
        %     distanceToImageEdges = min(distanceCentroidLeftImageBoundary,distanceCentroidRightImageBoundary,...
        %         distanceCentroidTopImageBoundary,distanceCentroidBottomImageBoundary);
        %     objectsCurrentImage(i).enteringExitingFOV = 1/distanceToImageEdges;
        
        %compute Euclidean distances
        previousImageCentroids = [previousImage.objects.centroid];
        previousImageCentroids = reshape(previousImageCentroids,...
            [size(previousImage.objects(1).centroid,2),nObjectPreviousImage])';
        distances = sqrt(sum(bsxfun(@minus,previousImageCentroids,objectCentroid).^2,2));
        scoreDistances = 20000.*1./distances;
        
        % for testing only
        %------------------------------------------------------------------
%         currentImage.objects(j).classGT
%         closestObjectClassGT = previousImage.objects(find(distances==min(distances))).classGT
%         HighestDistanceScoreObjectClassGT = previousImage.objects(find(scoreDistances==max(scoreDistances))).classGT
%         if~(strcmp(closestObjectClassGT,currentImage.objects(j).classGT))
%             disp('wrong closest distance')
%         end
%         disp('***********************************************************')
        %------------------------------------------------------------------
        
        %get Object Centroid Depth
        objectCentroidDepth = double(currentImage.depth(round(objectCentroid(2)),...
            round(objectCentroid(1))));
        %compute Euclidean distances
        previousImageCentroidsDepths = zeros(nObjectPreviousImage,1);
        for k=1:nObjectPreviousImage
            centroidDepth = previousImage.depth(round(previousImageCentroids(k,2)),...
            round(previousImageCentroids(k,1)));
            previousImageCentroidsDepths(k,1) = double(centroidDepth); 
        end
        depthDistances = sqrt(sum(bsxfun(@minus,previousImageCentroidsDepths,objectCentroidDepth).^2,2));
        scoreDepthDistances = 20000.*1./depthDistances; 
      
        %get predicted centroid
        applyPredictedCentroid = 0;
        if isfield(previousImage.objects,'predictedCentroid')      
            objectPredictedCentroids = [previousImage.objects.predictedCentroid];
            objectPredictedCentroids = reshape(objectPredictedCentroids,...
            [size(previousImage.objects(1).centroid,2),nObjectPreviousImage])';
        
            predictedCentroidDistances = sqrt(sum(bsxfun(@minus,objectCentroid,objectPredictedCentroids).^2,2));
            scorePredictedCentroidDistance = 20000.*1./predictedCentroidDistances; 
            applyPredictedCentroid = 1;
        end
        
        % match features in bounding boxes
        applyMatchFeatures = 1;
        currentBB = currentImage.objects(j).boundingBox;
        if isempty(currentBB)
            applyMatchFeatures = 0;
        else
        currentCroppedImage = currentImage.I(currentBB(2):currentBB(4),...
            currentBB(1):currentBB(3),:);
        currentPoints = detectSURFFeatures(rgb2gray(currentCroppedImage));
        [currentFeatures,~] = extractFeatures(rgb2gray(currentCroppedImage),currentPoints);
        end
        if isempty(currentFeatures)
            applyMatchFeatures = 0;
        end
        matchMetrics = 1000*ones(nObjectPreviousImage,1);
        for k=1:nObjectPreviousImage
            previousBB = previousImage.objects(k).boundingBox;
            if isempty(previousBB)
                applyMatchFeatures = 0;
            else
            previousCroppedImage = previousImage.I(previousBB(2):previousBB(4),...
                previousBB(1):previousBB(3),:);
            previousPoints = detectSURFFeatures(rgb2gray(previousCroppedImage));
            [previousFeatures,~] = extractFeatures(rgb2gray(previousCroppedImage),previousPoints);
            end
            if isempty(previousFeatures)
                applyMatchFeatures = 0;
            end
            if applyMatchFeatures == 1
                [~, matchMetric] = matchFeatures(currentFeatures,previousFeatures,'Metric','SSD');
                if ~isempty(matchMetric)
                    matchMetrics(k,1) = sum(matchMetric);
                else
                    applyMatchFeatures = 0;
                end
                if matchMetrics(k,1) == 0
                    matchMetrics(k,1) = 1e-6;
                end
            end
        end
        scoreMatchMetrics = 10.*1./matchMetrics;
        
        % for testing only
        %------------------------------------------------------------------
%         currentImage.objects(j).classGT
%         HighestMatchMetricScoreObjectClassGT = previousImage.objects(find(scoreMatchMetrics==max(scoreMatchMetrics))).classGT
%         if~(strcmp(HighestMatchMetricScoreObjectClassGT,currentImage.objects(j).classGT))
%             disp('wrong highest match metric')
%         end
%         disp('***********************************************************')
        %------------------------------------------------------------------
        if length(find(scoreMatchMetrics==max(scoreMatchMetrics))) > 1
           applyMatchFeatures = 0;
        end
        if applyMatchFeatures == 1 %&& previousImage.objects(find(scoreDistances==max(scoreDistances))).id ~=...
                %previousImage.objects(find(scoreMatchMetrics==max(scoreMatchMetrics))).id
            % scores
            scores = scoreDistances + scoreDepthDistances + scoreMatchMetrics;
        else
            scores = scoreDistances + scoreDepthDistances;
        end
        if applyPredictedCentroid
            scores = scores + scorePredictedCentroidDistance;
        end
        
        %find the highest score object
        highestRankObject = previousImage.objects(find(scores==max(scores)));
        if nObjectCurrentImage <= nObjectPreviousImage
            % assign object ID to closest object id
            if strcmp(highestRankObject.class,currentImage.objects(j).class)
                currentImage.objects(j).id = highestRankObject.id;
                
                % for testing only
                %------------------------------------------------------------------
                %currentImage.objects(j).classGT
                %highestRankObject.classGT
                if~(strcmp(highestRankObject.classGT,currentImage.objects(j).classGT))
                    wrongAssociations = wrongAssociations + 1;
                    disp('wrong highest rank')
                end
                disp('***********************************************************')
                %------------------------------------------------------------------
                
                % predict next object centroid position
                previousObjectCentroid = highestRankObject.centroid;
                deltaCentroid = objectCentroid - previousObjectCentroid;
                nextPredictedCentroid = objectCentroid + deltaCentroid;
                currentImage.objects(j).predictedCentroid = nextPredictedCentroid;
            end
            %% check if new object -- to be fixed
            %% only one object should get a maxID+1, others copy previous image ids
        elseif nObjectCurrentImage > nObjectPreviousImage
            currentImage.objects(j).id = maxID + 1;
        end
        % update max id
        if max([currentImage.objects.id]) > maxID
            maxID = max([currentImage.objects.id]);
        end
    end
    images(i) = currentImage;
end
end
