%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 17/01/18
% Contributors:
%--------------------------------------------------------------------------
function RSS18g(factor,noiseLevel)
%% 1. Config
% time
t0 = 1;
tN = 64*factor;
nSteps = 64*factor;
t = linspace(t0,tN,nSteps);
dt = (tN-(t0-1))/(nSteps);

config = CameraConfig();
config = setAppConfig(config);
if noiseLevel == 1  
    config.set('std2PointsSE3Motion', [0.01,0.01,0.01]');
elseif noiseLevel == 2
    config.set('std2PointsSE3Motion', [0.05,0.05,0.05]');
elseif noiseLevel == 3
    config.set('std2PointsSE3Motion', [0.1,0.1,0.1]');
end

% config.set('noiseModel','Off');
config.set('groundTruthFileName',strcat('RSS18g_groundTruth_',num2str(factor),...
    '_',num2str(noiseLevel),'.graph'));
config.set('measurementsFileName',strcat('RSS18g_measurements_',num2str(factor),...
    '_',num2str(noiseLevel),'.graph'));
config.set('t',t);

% SE3 Motion
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('motionModel','constantSE3MotionDA');
%% 2. Generate Environment
if config.rngSeed
    rng(config.rngSeed);
end

%% 2.1 robot waypoints and trajectory
robotWaypoints = [6 + sin(t * .5); 10 + cos(t * .5); (5 - 5 * (.5 + .5 * sin(t / 10)))]'; 
robotWaypoints = reshape(robotWaypoints',[size(robotWaypoints,2),size(robotWaypoints,1)]);
robotTrajectoryWaypoints = [t;robotWaypoints];
robotTrajectory = PositionModelPoseTrajectory(robotTrajectoryWaypoints,'R3','smoothingspline');
%% 2.2 objects waypoints and trajectories
nObjects = 0;
primitivesTrajectories = {};
% primitive 1
primitive1InitialPose_R3xso3 = [5 6 2 pi/3 0 pi/2]';
primitive1Motion_R3xso3 = [0.105*dt; 0; 0; 0.225*arot(eul2rot([0.105*dt,0,0]))];
primitive1Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive1InitialPose_R3xso3,primitive1Motion_R3xso3,'R3xso3');
nObjects = nObjects+1;
primitivesTrajectories{nObjects} = primitive1Trajectory;
% primitive 2
if factor>=2
primitive2InitialPose_R3xso3 = primitive1Trajectory.get('R3xso3Pose',t(64))+[5;5;5;0;0;0];
primitive2Motion_R3xso3 = -primitive1Motion_R3xso3;
primitive2Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive2InitialPose_R3xso3,primitive2Motion_R3xso3,'R3xso3');
nObjects = nObjects+1;
primitivesTrajectories{nObjects} = primitive2Trajectory;
end
% primitive 3
primitive3InitialPose_R3xso3 = [12 15 2 pi/3 0 -pi/2]';
primitive3Motion_R3xso3 = [-0.21*dt; 0; 0; 0.225*arot(eul2rot([-0.105*dt,0,0]))];
primitive3Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive3InitialPose_R3xso3,primitive3Motion_R3xso3,'R3xso3');
nObjects = nObjects+1;
primitivesTrajectories{nObjects} = primitive3Trajectory;
% primitive 4
if factor>=2
primitive4InitialPose_R3xso3 = primitive3Trajectory.get('R3xso3Pose',t(64))+[5;5;5;0;0;0];
primitive4Motion_R3xso3 = -primitive3Motion_R3xso3;
primitive4Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive4InitialPose_R3xso3,primitive4Motion_R3xso3,'R3xso3');
nObjects = nObjects+1;
primitivesTrajectories{nObjects} = primitive4Trajectory;
end
% primitive 5
primitive5InitialPose_R3xso3 = [4 8 3 pi/6 0 -pi/3]';
primitive5Motion_R3xso3 = (1/60*factor)*[1; 1.2; 0; arot(eul2rot([0,0,0]))];
primitive5Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive5InitialPose_R3xso3,primitive5Motion_R3xso3,'R3xso3');
nObjects = nObjects+1;
primitivesTrajectories{nObjects} = primitive5Trajectory;
% primitive 6
if factor>=2
primitive6InitialPose_R3xso3 = primitive5Trajectory.get('R3xso3Pose',t(64))+[5;5;5;0;0;0];
primitive6Motion_R3xso3 = -primitive5Motion_R3xso3;
primitive6Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive6InitialPose_R3xso3,primitive6Motion_R3xso3,'R3xso3');
nObjects = nObjects+1;
primitivesTrajectories{nObjects} = primitive6Trajectory;
end
% primitive 7
primitive7InitialPose_R3xso3 = [8 12 6 pi/3 0 -pi/6]';
primitive7Motion_R3xso3 = (1/60*factor)*[-0.2; 1; 0; arot(eul2rot([0,0,0]))];
primitive7Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive7InitialPose_R3xso3,primitive7Motion_R3xso3,'R3xso3');
nObjects = nObjects+1;
primitivesTrajectories{nObjects} = primitive7Trajectory;
% primitive 8
if factor>=2
primitive8InitialPose_R3xso3 = primitive7Trajectory.get('R3xso3Pose',t(64))+[5;5;5;0;0;0];
primitive8Motion_R3xso3 = -primitive7Motion_R3xso3;
primitive8Trajectory = ConstantMotionDiscretePoseTrajectory(t,...
    primitive8InitialPose_R3xso3,primitive8Motion_R3xso3,'R3xso3');
nObjects = nObjects+1;
primitivesTrajectories{nObjects} = primitive8Trajectory;
end
%% constant SE3 motion
constantSE3ObjectMotion = zeros(6,nObjects);
for i=1:nObjects
    constantSE3ObjectMotion(:,i) = primitivesTrajectories{i}.RelativePoseGlobalFrameR3xso3(t(1),t(2));
end

%% construct environment
environment = Environment();
for i=1:nObjects
    environment.addEllipsoid([1 1 2.5],8,'R3',primitivesTrajectories{i});
end

%% 3. Initialise Sensor
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);
% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

figure
spy(sensor.get('pointVisibility'));
%% 4. Plot Environment
% figure
% viewPoint = [-35,35];
% % axisLimits = [-30,30,-5,30,-10,10];
% % title('Environment')
% axis equal
% xlabel('x (m)')
% ylabel('y (m)')
% zlabel('z (m)')
% view(viewPoint)
% % axis(axisLimits)
% hold on
% grid on
% for i=1:nObjects
%     primitivesTrajectories{i}.plot(t,[0 0 0],'axesOFF')
%     hold on
% end
% cameraTrajectory.plot(t,[0 0 1],'axesOFF')
% % set(gcf,'Position',[0 0 1024 768]);
% frames = sensor.plot(t,environment);
% implay(frames);


%% 4.a output video
% v = VideoWriter('Data/Videos/App6_sensor_environment.mp4','MPEG-4');
% open(v)
% writeVideo(v,frames);
% close(v)

%% 5. Generate Measurements & Save to Graph File, load graph file as well
config.set('constantSE3Motion',constantSE3ObjectMotion);
%% 5.1 For initial (without SE3)
config.set('pointMotionMeasurement','Off')
config.set('measurementsFileName',strcat('RSS18g_measurementsNoSE3_',num2str(factor),'_',...
    num2str(noiseLevel),'.graph'))
config.set('groundTruthFileName',strcat('RSS18g_groundTruthNoSE3_',num2str(factor),'_',...
    num2str(noiseLevel),'.graph'))
sensor.generateMeasurements(config);
groundTruthNoSE3Cell = graphFileToCell(config,config.groundTruthFileName);
measurementsNoSE3Cell = graphFileToCell(config,config.measurementsFileName);

%% 5.2 For test (with SE3)
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('measurementsFileName',strcat('RSS18g_measurements_',num2str(factor),'_',...
    num2str(noiseLevel),'.graph'));
config.set('groundTruthFileName',strcat('RSS18g_groundTruth_',num2str(factor),'_',...
    num2str(noiseLevel),'.graph'));
sensor.generateMeasurements(config);
writeDataAssociationVerticesEdges_constantSE3Motion(config,constantSE3ObjectMotion);
measurementsCell = graphFileToCell(config,config.measurementsFileName);
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);

%% 6. Solve
% config.set('sortVertices',1);
% config.set('sortEdges',1);
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
initialGraphN.saveGraphFile(config,strcat('RSS18g_resultsNoSE3_',num2str(factor),'_',...
    num2str(noiseLevel),'.graph'));

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
graphN.saveGraphFile(config,strcat('RSS18g_results_',num2str(factor),'_',...
    num2str(noiseLevel),'.graph'));

%% 7. Error analysis
%load ground truth into graph, sort if required
graphGT = Graph(config,groundTruthCell);
graphGTNoSE3 = Graph(config,groundTruthNoSE3Cell);
fprintf('\nInitial results for without SE(3) Transform:\n')
resultsNoSE3 = errorAnalysis(config,graphGTNoSE3,initialGraphN);
fprintf('\nFinal results for SE(3) Transform:\n')
resultsSE3 = errorAnalysis(config,graphGT,graphN);

% save results
H = solverEnd.systems(end).H;
save(strcat('Data/RSS18g_results/resultsNoSE3_',num2str(factor),'_',num2str(noiseLevel)),'resultsNoSE3')
save(strcat('Data/RSS18g_results/resultsSE3_',num2str(factor),'_',num2str(noiseLevel)),'resultsSE3')
save(strcat('Data/RSS18g_results/H_',num2str(factor),'_',num2str(noiseLevel)),'H')

%% 8. Plot
%% 8.1 Plot initial, final and ground-truth solutions
%no constraints
% figure
% spy(solverEnd.systems(end).H)

% figure
% spy(chol(solverEnd.systems(end).H))

% h = figure;
% xlabel('x (m)')
% ylabel('y (m)')
% zlabel('z (m)')
% hold on
% grid on
% axis equal
% axisLimits = [-25,25,0,30,-5,15];
% axis(axisLimits)
% view([-50,25])
% %plot groundtruth
% plotGraphFileICRA(config,groundTruthCell,'groundTruth');
% %plot results
% resultsNoSE3Cell = graphFileToCell(config,'RSS18f_resultsNoSE3.graph');
% resultsCell = graphFileToCell(config,'RSS18f_results.graph');
% plotGraphFileICRA(config,resultsNoSE3Cell,'initial',resultsNoSE3.relPose.get('R3xso3Pose'),...
%     resultsNoSE3.posePointsN.get('R3xso3Pose'))
% plotGraphFileICRA(config,resultsCell,'solverResults',resultsSE3.relPose.get('R3xso3Pose'),...
%     resultsSE3.posePointsN.get('R3xso3Pose'))
end