%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 30/10/2018
% Testing dynamic parallel SLAM + MOT
%--------------------------------------------------------------------------
%% 1. Config
% time
t0 = 0;
nSteps = 21;
tN = 20;
dt = (tN-t0)/(nSteps-1);
t  = linspace(t0,tN,nSteps);

config = CameraConfig();
config = setAppConfig(config); 
config.set('mode','parallel');
config.set('t',t);
config.set('nSteps',nSteps);
% config.set('noiseModel','Off');
config.set('groundTruthFileName','Test_DynamciSLAM_parallel_groundTruth.graph');
config.set('measurementsFileName','Test_DynamciSLAM_parallel_measurements.graph');

% SE3 Motion
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.5,0.5,0.5]');
config.set('SE3MotionVertexInitialization','eye');
config.set('newMotionVertexPerNLandmarks',inf);
config.set('landmarksSlidingWindowSize',inf);
config.set('objectPosesSlidingWindow',false);
config.set('objectPosesSlidingWindowSize',inf);
config.set('newMotionVertexPerNObjectPoses',inf);
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('pointsDataAssociationLabel','2PointsDataAssociation');


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

environment = Environment();
environment.addEllipsoid([5 2 3],8,'R3',primitiveTrajectory);
environment.addStaticPoints([40*ones(1,80); 20*rand(2,80)]);
environment.addStaticPoints([20*rand(1,80); 65*ones(1,80); 20*rand(1,80)]);
environment.addStaticPoints([-30*ones(1,80); 20*rand(2,80)]);

%% 3. Initialise Sensor
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);
% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

% figure
% spy(sensor.get('pointVisibility'));
% 
%% 4. Plot Environment
% figure
% hold on
% grid on
% axis equal
% viewPoint = [-50,25];
% axisLimits = [-30,50,-10,70,-10,25];
% axis equal
% xlabel('x (m)')
% ylabel('y (m)')
% zlabel('z (m)')
% view(viewPoint)
% axis(axisLimits)
% primitiveTrajectory.plot(t,[0 0 0],'axesOFF')
% cameraTrajectory.plot(t,[0 0 1],'axesOFF')
% frames = sensor.plot(t,environment);
% % implay(frames);
    
%% 5. Generate Measurements & Save to Graph File
sensor.generateMeasurements(config);

%% 6. load graph files
writeDataAssociationObjectIndices(config,1)
config.set('measurementsFileName',strcat(config.measurementsFileName(1:end-6),'Test.graph'));
config.set('groundTruthFileName',strcat(config.groundTruthFileName(1:end-6),'Test.graph'));

measurementsCell = graphFileToCell(config,config.measurementsFileName);
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);

%% 7. Solve
%no constraints
timeStart = tic;
graph0 = Graph();
solver = graph0.process(config,measurementsCell,groundTruthCell);
solverStatic = solver(1:length(solver)/2);
solverDynamic = solver(length(solver)/2+1:end);
solverStaticEnd = solverStatic(end);
solverDynamicEnd = solverDynamic(end);
totalTime = toc(timeStart);
fprintf('\nTotal time solving: %f\n',totalTime)

%get desired graphs & systems
graphNStatic  = solverStaticEnd.graphs(end);
graphNDynamic = solverDynamicEnd.graphs(end);

posesStatic  = [graphNStatic.vertices(graphNStatic.identifyVertices('pose')).value];
posesDynamic  = [graphNDynamic.vertices(graphNDynamic.identifyVertices('pose')).value];
%save results to graph file
graphNStatic.saveGraphFile(config,'Test_DynamciSLAM_parallel_staticResults.graph');
graphNDynamic.saveGraphFile(config,'Test_DynamciSLAM_parallel_dynamicResults.graph');

%% 8. Error analysis
%load ground truth into graph, sort if required
graphGT = Graph(config,groundTruthCell);
fprintf('Static results error: \n');
staticResults = errorAnalysis(config,graphGT,graphNStatic);
fprintf('Dynamic results error: \n');
dynamicResults = errorAnalysis(config,graphGT,graphNDynamic);

%% 9. Plot
    %% 10.1 Plot intial, final and ground-truth solutions
figure
subplot(1,2,1)
spy(solverStaticEnd.systems(end).A)
subplot(1,2,2)
spy(solverStaticEnd.systems(end).H)

figure
subplot(1,2,1)
spy(solverDynamicEnd.systems(end).A)
subplot(1,2,2)
spy(solverDynamicEnd.systems(end).H)

h = figure; 
xlabel('x')
ylabel('y')
zlabel('z')
hold on
grid on
view([-50,25])
%plot groundtruth
plotGraphFile(config,groundTruthCell,[0 0 1]);
%plot results
staticResultsCell = graphFileToCell(config,'Test_DynamciSLAM_parallel_staticResults.graph');
plotGraphFile(config,staticResultsCell,[1 0 0])
hold on
dynamicPoints = [graphNDynamic.vertices(graphNDynamic.identifyVertices('point')).value];
plotPoints = plot3(dynamicPoints(1,:),dynamicPoints(2,:),dynamicPoints(3,:),'.');
set(plotPoints,'MarkerEdgeColor',[0 1 0])
set(plotPoints,'MarkerSize',8)
