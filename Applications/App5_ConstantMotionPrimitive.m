%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 03/07/2017
% Contributors:
%--------------------------------------------------------------------------
% frontEndSolverExample
clear all 
% close all 

%% 1. Config
% time
nSteps = 121;
t0 = 0;
tN = 120;
dt = (tN-t0)/(nSteps-1);
t  = linspace(t0,tN,nSteps);

config = CameraConfig();
config = setAppConfig(config); % copy same settings for error Analysis
% config = setLowErrorAppConfig(config);
%config = setHighErrorAppConfig(config);
config.set('t',t);
config.set('nSteps',nSteps);
% config.set('noiseModel','Off');
config.set('groundTruthFileName','app5_groundTruth.graph');
config.set('measurementsFileName','app5_measurements.graph');
%config.set('mode','initialisation');
objectAware = 1;

% SE3 Motion
if objectAware
    config.set('motionModel','constantSE3MotionDA');
    config.set('std2PointsSE3Motion', [0.01,0.01,0.01]');
    config.set('SE3MotionVertexInitialization','eye');
    config.set('newMotionVertexPerNLandmarks',inf);
    config.set('landmarksSlidingWindowSize',inf);
    config.set('objectPosesSlidingWindow',false);
    config.set('objectPosesSlidingWindowSize',inf);
    config.set('newMotionVertexPerNObjectPoses',inf);
else
    config.set('motionModel','constantVelocity');
    config.set('std2PointsVelocity', [0.01,0.01,0.01]');    
end

%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed); 
end

% construct primitive trajectory
primitiveInitialPose_R3xso3 = [10 0 0 0 0 0.2]';
primitiveMotion_R3xso3 = [1.5*dt; 0; 0; arot(eul2rot([0.05*dt,0,0.005*dt]))];
primitiveTrajectory = ConstantMotionDiscretePoseTrajectory(t,primitiveInitialPose_R3xso3,primitiveMotion_R3xso3,'R3xso3');

% construct  robot trajectories
sampleTimes = t(1:floor(numel(t)/5):numel(t));
sampleWaypoints = primitiveTrajectory.get('R3xso3Pose',sampleTimes);
robotWaypoints = [linspace(0,tN+3,numel(sampleTimes)+1); 0 sampleWaypoints(1,:); 0 (sampleWaypoints(2,:)+0.1); 0 (sampleWaypoints(3,:)-0.1)];
robotTrajectory = PositionModelPoseTrajectory(robotWaypoints,'R3','smoothingspline');

constantSE3ObjectMotion = primitiveTrajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));

environment = Environment();
environment.addEllipsoid([5 2 3],8,'R3',primitiveTrajectory);

%% 3. Initialise Sensor
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

% standard sensor
% sensor = SimulatedEnvironmentSensor();
% sensor.addEnvironment(environment);
% sensor.addCamera(config.fieldOfView,cameraTrajectory);
% sensor.setVisibility(config);

% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
% for a transparent object use:
% sensor = SimulatedEnvironmentSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config, environment);

% figure
% spy(sensor.get('pointVisibility'));
% 
% %% 4. Plot Environment
% figure
% hold on
% grid on
% axis equal
% viewPoint = [-50,25];
% axisLimits = [-30,50,-10,60,-10,25];
% axis equal
% xlabel('x (m)')
% ylabel('y (m)')
% zlabel('z (m)')
% view(viewPoint)
% axis(axisLimits)
% primitiveTrajectory.plot(t,[0 0 0],'axesOFF')
% cameraTrajectory.plot(t,[0 0 1],'axesOFF')
% % set(gcf,'Position',[0 0 1024 768]);
% frames = sensor.plot(t,environment);
% implay(frames);


%% 4.a output video
% v = VideoWriter('Data/Videos/App5_sensor_environment.mp4','MPEG-4');
% open(v)
% writeVideo(v,frames);
% close(v)

%% 5. Generate Measurements & Save to Graph File, load graph file as well
config.set('constantSE3Motion',constantSE3ObjectMotion);
    %% 5.1 For initial (without SE3)
    config.set('pointMotionMeasurement','Off')
    config.set('measurementsFileName','app5_measurementsNoSE3.graph')
    config.set('groundTruthFileName','app5_groundTruthNoSE3.graph')
    if config.rngSeed
        rng(config.rngSeed); 
    end
    sensor.generateMeasurements(config);
    groundTruthNoSE3Cell = graphFileToCell(config,config.groundTruthFileName);
    measurementsNoSE3Cell = graphFileToCell(config,config.measurementsFileName);
    
    %% 5.2 For test (with SE3)    
    if objectAware
        config.set('pointMotionMeasurement','point2DataAssociation');
    else
        config.set('pointMotionMeasurement','velocity');
    end
    config.set('pointsDataAssociationLabel','2PointsDataAssociation');
    config.set('measurementsFileName','app5_measurements.graph');
    config.set('groundTruthFileName','app5_groundTruth.graph');
    if config.rngSeed
        rng(config.rngSeed); 
    end
    sensor.generateMeasurements(config);
    
    if objectAware
        writeDataAssociationObjectIndices(config,1)
        config.set('measurementsFileName',...
            strcat(config.measurementsFileName(1:end-6),'Test.graph'));
        config.set('groundTruthFileName',...
            strcat(config.groundTruthFileName(1:end-6),'Test.graph')); 
    end
%     writeDataAssociationVerticesEdges_constantSE3Motion(config,constantSE3ObjectMotion);
%     config.set('measurementsFileName',...
%             strcat(config.measurementsFileName(1:end-6),'Test.graph'));
%         config.set('groundTruthFileName',...
%             strcat(config.groundTruthFileName(1:end-6),'Test.graph'));
    measurementsCell = graphFileToCell(config,config.measurementsFileName);
    groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);

%% 6. Solve
    %% 6.1 Without SE3
    timeStart = tic;
    initialGraph0 = Graph();
    initialSolver = initialGraph0.process(config,measurementsNoSE3Cell,groundTruthNoSE3Cell);
    initialSolverEnd = initialSolver(end);
    totalTime = toc(timeStart);
    fprintf('\nTotal time solving: %f\n',totalTime)

    %get desired graphs & systems
    initialGraph0  = initialSolverEnd.graphs(1);
    initialGraphN  = initialSolverEnd.graphs(end);
    %save results to graph file
    initialGraphN.saveGraphFile(config,'app5_resultsNoSE3.graph');
    
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
    graphN.saveGraphFile(config,'app5_results.graph');

%% 7. Error analysis
%load ground truth into graph, sort if required
graphGTNoSE3 = Graph(config,groundTruthNoSE3Cell);
graphGT = Graph(config,groundTruthCell);
fprintf('\nInitial results for without SE(3) Transform:\n')
resultsNoSE3 = errorAnalysis(config,graphGTNoSE3,initialGraphN);
fprintf('\nFinal results for SE(3) Transform:\n')
resultsSE3 = errorAnalysis(config,graphGT,graphN);

%% 8. Plot
    %% 8.1 Plot initial, final and ground-truth solutions
%no constraints
figure
subplot(1,2,1)
spy(solverEnd.systems(end).A)
subplot(1,2,2)
spy(solverEnd.systems(end).H)

h = figure; 
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
hold on
grid on
axis equal
axisLimits = [-30,50,-10,60,-25,25];
axis(axisLimits)
view([-50,25])
%plot groundtruth
plotGraphFileICRA(config,groundTruthCell,'groundTruth');
%plot results
resultsNoSE3Cell = graphFileToCell(config,'app5_resultsNoSE3.graph');
resultsCell = graphFileToCell(config,'app5_results.graph');
plotGraphFileICRA(config,resultsNoSE3Cell,'initial',resultsNoSE3.relPose.get('R3xso3Pose'),resultsNoSE3.posePointsN.get('R3xso3Pose'))
% get indices of static and dynamic points per object
dynamicPointsVertices = {};
allDynamicPointsVertices = [];
SE3MotionVertices = [graphN.identifyVertices('SE3Motion')];
pointIndices = [graphN.identifyVertices('point')];
for i=1:numel(SE3MotionVertices)
    edgesConnectedToMotionVertex = [graphN.vertices(SE3MotionVertices(i)).iEdges];
    dynamicPointsMotionIndices = [graphN.edges(edgesConnectedToMotionVertex).iVertices]';
    dynamicPointsIndices = setdiff(dynamicPointsMotionIndices,SE3MotionVertices);
    dynamicPointsVertices{i} = dynamicPointsIndices;
    allDynamicPointsVertices = [allDynamicPointsVertices,dynamicPointsIndices'];
end
staticPointsIndices = setdiff(pointIndices,allDynamicPointsVertices);
plotGraphFileICRA(config,resultsCell,'solverResults',resultsSE3.relPose.get('R3xso3Pose'),resultsSE3.posePointsN.get('R3xso3Pose'),graphN,[staticPointsIndices dynamicPointsVertices])