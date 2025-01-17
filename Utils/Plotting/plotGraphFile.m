function plotGraphFile(config,groundTruthCell,graphColour)
%PLOTGRAPHFILE Summary of this function goes here
%   Detailed explanation goes here

%% 1. get vertices and labels
rowLengths = cellfun('length',groundTruthCell);
vertexRows = (rowLengths==3);
verticesCellTemp = groundTruthCell(vertexRows,:);
verticesCell = cell(sum(vertexRows),3);
for i = 1:sum(vertexRows)
    verticesCell(i,:) = verticesCellTemp{i,:};
end
vertexLabels = verticesCell(:,1);

%% 2. get edges and labels
edgeRows = (rowLengths==6);
edgesCellTemp = groundTruthCell(edgeRows,:);
edgesCell = cell(sum(edgeRows),6);
for i = 1:sum(edgeRows)
    edgesCell(i,:) = edgesCellTemp{i,:};
end
edgeLabels = edgesCell(:,1);

%% 3. identify poses, points, planes, point
poseVertices  = strcmp(vertexLabels,config.poseVertexLabel);
pointVertices = strcmp(vertexLabels,config.pointVertexLabel);
planeVertices = strcmp(vertexLabels,config.planeVertexLabel);
pointPlaneEdges = strcmp(edgeLabels,config.pointPlaneEdgeLabel);

%% 4. get values
poses  = [verticesCell{poseVertices,3}];
points = [verticesCell{pointVertices,3}];
planes = [verticesCell{planeVertices,3}];

%% 5. plot poses
plot3(poses(1,:),poses(2,:),poses(3,:),'Color',graphColour,'Marker','o','LineStyle','none');
for i = 1:sum(poseVertices)
    iPose = poses(:,i);
    if strcmp(config.poseParameterisation,'SE3')
        iPose = LogSE3_Rxt(iPose);
    end
    
    scale = 0.2;
    plotCoordinates(iPose(1:3,:),scale*rot(iPose(4:6,1)))
%     plotiCamera = plotCamera('Location',iPose(1:3),'Orientation',rot(-iPose(4:6))); %LHS invert pose
%     scale = 0.5;
%     plotCoordinates(iPose(1:3),scale*rot(iPose(4:6)))
%     plotiCamera.Opacity = 0.1;
%     plotiCamera.Size = 0.1;
%     plotiCamera.Color = graphColour;
end

%% 6. plot points
plotPoints = plot3(points(1,:),points(2,:),points(3,:),'.');
set(plotPoints,'MarkerEdgeColor',graphColour)
set(plotPoints,'MarkerSize',8)
% set(plotPoints,'MarkerSize',8)
% axis equal
%% 7. plot planes
%need edges to figure out which points are on which planes
iPlaneVertices = [verticesCell{planeVertices,2}];
pointPlaneVertices = [edgesCell{pointPlaneEdges,3}; %[point plane]
                      edgesCell{pointPlaneEdges,4}]';
for i = 1:sum(planeVertices)
    %plane vertex index
    iPlaneVertex = iPlaneVertices(i);
    
    %identify plane points
    iPlanePointVertices = unique(pointPlaneVertices(pointPlaneVertices(:,2)==iPlaneVertex,1));
    iPlanePointPositions = [verticesCell{iPlanePointVertices,3}];
    
    %plot plane points
%     plotPlanePoints(i) = plot3(iPlanePointPositions(1,:),iPlanePointPositions(2,:),iPlanePointPositions(3,:),'*');
%     set(plotPlanePoints(i),'MarkerEdgeColor',graphColour)
%     set(plotPlanePoints(i),'MarkerSize',5)
    
    %plot plane
    if config.plotPlanes
        planeParameters = planes(:,i);
        normal = planeParameters(1:3);
        distance = planeParameters(4);
        plotPlane(normal,distance,iPlanePointPositions,graphColour);
    end
end

end

