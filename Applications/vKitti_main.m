%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 01/02/2018
% Contributors:
%--------------------------------------------------------------------------
% vKitti main
% 1. Config
% objectPosesMatrix = 'objPose_0001_334_425.mat';
% [objectsGTMotion, objectsGTFrames] = vKitti_objectMotion(objectPosesMatrix);
% constantSE3ObjectMotion = vKitti_objectMotionAveraged(objectPosesMatrix);
% nObjects = size(constantSE3ObjectMotion,2);

config = CameraConfig();
config = setAppConfig(config);
% config.set('noiseModel','Off');
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [1,1,1]');
config.set('SE3MotionVertexInitialization','eye');
config.set('newMotionVertexPerNLandmarks',inf);
config.set('landmarksSlidingWindowSize',inf);
config.set('objectPosesSlidingWindow',false);
config.set('objectPosesSlidingWindowSize',inf);
config.set('newMotionVertexPerNObjectPoses',inf);
% config.set('robustCostFunction','GemanMcClure')
% config.set('robustCostFunctionWidth',3)

%% 5. Generate Measurements & Save to Graph File, load graph file as well
%% 5.1 For initial (without SE3)
% config.set('pointMotionMeasurement','Off')
% config.set('measurementsFileName','staticDynamic92ImagesNoSE3Meas.graph')
% config.set('groundTruthFileName','staticDynamic92ImagesNoSE3GT.graph')
% groundTruthNoSE3Cell = graphFileToCell(config,config.groundTruthFileName);
% measurementsNoSE3Cell = graphFileToCell(config,config.measurementsFileName);

%% 5.2 For test (with SE3)
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('pointsDataAssociationLabel','2PointsDataAssociation');
config.set('measurementsFileName','finalNoiseSequence0001_334to426_final_Meas.graph');
config.set('groundTruthFileName','finalNoiseSequence0001_334to426_final_GT.graph'); 
% Check for wrong data associations and fix if necessary
% dataAssociationTest(config,config.measurementsFileName,nObjects);
% dataAssociationTest(config,config.groundTruthFileName,nObjects);
% writeDataAssociationObjectIndices(config,nObjects)
% writeDataAssociationVerticesEdges_constantSE3Motion(config,constantSE3ObjectMotion);
% config.set('measurementsFileName',...
%     strcat(config.measurementsFileName(1:end-6),'Test.graph'));
% config.set('groundTruthFileName',...
%     strcat(config.groundTruthFileName(1:end-6),'Test.graph')); 
% corruptDataAssociation(config,0.3);
% config.set('measurementsFileName',...
%     strcat(config.measurementsFileName(1:end-6),'Corrupted.graph'));
measurementsCell = graphFileToCell(config,config.measurementsFileName);
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);

%% 6. Solve
%% 6.1 Without SE3
% timeStart = tic;
% initialGraph0 = Graph();
% initialSolver = initialGraph0.process(config,measurementsNoSE3Cell,groundTruthNoSE3Cell);
% initialSolverEnd = initialSolver(end);
% totalTime = toc(timeStart);
% fprintf('\nTotal time solving: %f\n',totalTime)
% %get desired graphs & systems
% initialGraph0  = initialSolverEnd.graphs(1);
% initialGraphN  = initialSolverEnd.graphs(end);
% %save results to graph file
% initialGraphN.saveGraphFile(config,'vKitti_resultsNoSE3.graph');

%% 6.2 With SE3
%no constraints
timeStart = tic;
graph0 = Graph();
solver = graph0.process(config,measurementsCell,groundTruthCell);
solverEnd = solver(end);
totalTime = toc(timeStart);
fprintf('\nTotal time solving: %f\n',totalTime)
%get desired graphs & systems
graph0  = solverEnd.graphs(1);
graphN  = solverEnd.graphs(end);
%save results to graph file
graphN.saveGraphFile(config,'Sequence0001_334to426_objectMotion_results.graph');

%% 7. Error analysis
%load ground truth into graph, sort if required
% graphGTNoSE3 = Graph(config,groundTruthNoSE3Cell);
% fprintf('\nInitial results for without SE(3) Transform:\n')
% resultsNoSE3 = errorAnalysis(config,graphGTNoSE3,initialGraphN);
graphGT = Graph(config,groundTruthCell);
fprintf('\nFinal results for SE(3) Transform:\n')
resultsSE3 = errorAnalysis(config,graphGT,graphN);

%% 8. Plot
%% 8.1 Plot initial, final and ground-truth solutions
%no constraints
% figure
% subplot(1,2,1)
% spy(solverEnd.systems(end).A)
% subplot(1,2,2)
% spy(solverEnd.systems(end).H)


figure('units','normalized','color','w');
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
hold on
grid on
axis equal
% axisLimits = [-30,50,-10,60,-25,25];
% axis(axisLimits)
view([-50,25])
%plot groundtruth
plotGraphFileICRA(config,groundTruthCell,'groundTruth');
%plot results
% resultsNoSE3Cell = graphFileToCell(config,'vKitti_resultsNoSE3.graph');
resultsCell = graphFileToCell(config,'Sequence0001_334to426_objectMotion_results.graph');
% plotGraphFileICRA(config,resultsNoSE3Cell,'initial',...
%     resultsNoSE3.relPose.get('R3xso3Pose'),resultsNoSE3.posePointsN.get('R3xso3Pose'))
plotGraphFileICRA(config,resultsCell,'solverResults',...
resultsSE3.relPose.get('R3xso3Pose'),resultsSE3.posePointsN.get('R3xso3Pose'),graphN)

% %plot heat map style points error
% pointsGT = [graphGT.vertices(graphGT.identifyVertices('point')).value];
% pointsN = [graphN.vertices(graphN.identifyVertices('point')).value];
% NoSE3_pointsN = [initialGraphN.vertices(initialGraphN.identifyVertices('point')).value];
% 
% posesGT = [graphGT.vertices(graphGT.identifyVertices('pose')).value];
% posesN = [graphN.vertices(graphN.identifyVertices('pose')).value];
% NoSE3_posesN = [initialGraphN.vertices(initialGraphN.identifyVertices('pose')).value];
% 
% plotHeatMapStylePoints(pointsGT,pointsN,NoSE3_pointsN,...
%     resultsSE3.posePointsN_SE3,resultsNoSE3.posePointsN_SE3,...
%     posesGT,posesN, NoSE3_posesN,resultsSE3.relPose.get('R3xso3Pose'),...
%     resultsNoSE3.relPose.get('R3xso3Pose'))
