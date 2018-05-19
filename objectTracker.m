function images = objectTracker(images)

for i=2:length(images)
    
    currentImage = images(i);
    previousImage = images(i-1);
    maxID = max([previousImage.objects.id]);
    nObjectCurrentImage  = currentImage.nObjects;
    nObjectPreviousImage  = previousImage.nObjects;
    
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
        %find the smallest distance
        closestObject = previousImage.objects(find(distances==min(distances)));
        if nObjectCurrentImage <= nObjectPreviousImage
            % assign object ID to closest object id
            if strcmp(closestObject.class,currentImage.objects(j).class) ...
                    %&& sqrt(sum((closestObject.centroid-objectCentroid).^2)) < 25
                currentImage.objects(j).id = closestObject.id;
            end
            %match features between bounding boxes
            %currentPoints = detectSURFFeatures(currentBB);
            %previousPoints = detectSURFFeatures(previousBB);
            %[currentFeatures,~] = extractFeatures(currentBB,currentPoints);
            %[previousFeatures,~] = extractFeatures(previousBB,previousPoints);
            %[indexPairs, matchMetric] = matchFeatures(currentFeatures,previousFeatures,'Metric','SSD');
            %norm(matchMetric)< threshold 
            
        % check if new object
        elseif nObjectCurrentImage > nObjectPreviousImage && ...
                sqrt(sum((closestObject.centroid-objectCentroid).^2)) >= 25
            currentImage.objects(j).id = maxID + 1;
        end
    end
    images(i) = currentImage;
end
end
