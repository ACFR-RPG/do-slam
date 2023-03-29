clear all 
close all 

%% 1. Config
% time
nSteps = 32;
t0 = 1;
tN = 32;
dt = (tN-t0)/(nSteps-1);
t  = linspace(t0,tN,nSteps);

config = CameraConfig();
config = setAppConfig(config); 
config.set('t',t);
config.set('groundTruthFileName','VDO_toy_long_groundTruth.graph');
config.set('measurementsFileName','VDO_toy_long_measurements.graph');

% SE3 Motion
config.set('pointMotionMeasurement','point2DataAssociation');
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.05,0.05,0.05]');

% config.set('fieldofView', [-pi/2,pi/2,-pi/3,pi/3,1,40]); % Requires update
% neg90z_rotm = eul2rotm([0, 0, pi/2]);

config.set('cameraRelativePose', GP_Pose([0;0;0;arot(eul2rot([0, 0, 0]))]));
% Camera frame convention - z outward, x downward, y to the left

%% Graph File labels

% config.set('poseVertexLabel'     ,'VERTEX_POSE_R3_SO3');
config.set('poseVertexLabel'     ,'VERTEX_SE3:QUAT');
% config.set('posePoseEdgeLabel'   ,'EDGE_R3_SO3');
config.set('posePoseEdgeLabel'   ,'EDGE_SE3:QUAT');
% config.set('posePointEdgeLabel'  ,'EDGE_3D');
config.set('posePointEdgeLabel'  ,'EDGE_SE3_TRACKXYZ');
% config.set('pointVertexLabel'    ,'VERTEX_POINT_3D');
config.set('pointVertexLabel'    ,'VERTEX_TRACKXYZ');
% config.set('pointsDataAssociationLabel','point2DataAssociation')
config.set('pointsDataAssociationLabel','EDGE_SE3_MOTION')

config.set('pointSE3MotionEdgeLabel','EDGE_2POINTS_SE3Motion');
config.set('planeVertexLabel'    ,'VERTEX_PLANE_4D');
config.set('pointPlaneEdgeLabel' ,'EDGE_1D');
config.set('pointPointEdgeLabel' ,'EDGE_2POINTS');
config.set('point3EdgeLabel','EDGE_3POINTS');
config.set('velocityVertexLabel','VERTEX_VELOCITY');
config.set('pointVelocityEdgeLabel','EDGE_2POINTS_VELOCITY');
config.set('SE3MotionVertexLabel','VERTEX_SE3Motion');
config.set('pointSE3MotionEdgeLabel','EDGE_2POINTS_SE3Motion');
config.set('posePriorEdgeLabel','EDGE_6D');

%% Adjust standard deviation

rot = eul2rot([pi/180,pi/180,pi/180]); % 1 degree position error
orientation = arot(rot);
config.set('stdPosePose'  ,[0.03,0.03,0.03,orientation']');
config.set('stdPosePoint' ,[0.1,0.1,0.1]');

%% Environment Parameter

Centre = [5; 10; 15]; % Centre (roughly) of the environment, where the traj sits
Scale = 50; % Roughly the half size of the Bounding Box of the environment, a radius
Ascend = 0; % How much increase in z 

staticDim = 1; % How many static points there are on each side (Top, Bottom, Left and Right) in one set
staticLen = 5; % How many sets of static points there are along the traj
staticRes = 5; % How far apart the static points are on each side
% Are there static point on this side (top, bottom, left and right)
isStaticTop = true;
isStaticBottom = true;
isStaticLeft = true;
isStaticRight = true;

%% Generate Environment

robotInitialPose_rotm = eye(3);
robotInitialPose_pos = [0; -Scale; 0] + Centre;
robotInitialPose_SE3 = [robotInitialPose_rotm, robotInitialPose_pos; 0, 0, 0, 1];
robotMotion_rotm = eul2rot([pi/nSteps, 0, 0]);
robotMotion_SE3 = [robotMotion_rotm, [pi*Scale/nSteps; 0; Ascend/nSteps]; 0, 0, 0, 1];

robotTrajectory = ConstantMotionDiscretePoseTrajectory(t,robotInitialPose_SE3,robotMotion_SE3,'SE3');
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

environment = Environment();

% Static Points

staticX = repmat(linspace(1, staticRes*(staticDim-1)+1, staticDim), 1, staticDim) - 25;
staticY = 10*ones(1, staticDim^2);

staticTZ = reshape(repmat(linspace(0, Ascend, staticLen), staticDim, 1), [1, staticLen*staticDim])... 
    + 10 + Centre(3); % [Z1, Z1, Z1, Z2, Z2, Z2...]
staticBZ = reshape(repmat(linspace(0, Ascend, staticLen), staticDim, 1), [1, staticLen*staticDim])...
    - 10 + Centre(3); % [Z1, Z1, Z1, Z2, Z2, Z2...]
staticLRZ = repmat(linspace(-staticRes, staticRes, staticDim), 1, staticLen)...
    + reshape(repmat(linspace(0, Ascend, staticLen), staticDim, 1), [1, staticLen*staticDim])...
    + Centre(3); % [Z1, Z2, Z3, Z1, Z2, Z3...]

staticTBX = reshape(repmat(cos(linspace(-pi/2, pi/2, staticLen)), staticDim, 1), [1, staticLen*staticDim])...
    .*(repmat(linspace(-staticRes, staticRes, staticDim), 1, staticLen) + Scale) + Centre(1); % [X1, X2, X3, X1, X2, X3...]
staticLX = reshape(repmat(cos(linspace(-pi/2, pi/2, staticLen)), staticDim, 1), [1, staticLen*staticDim])...
    *(Scale - 10) + Centre(1); % [X1, X1, X1, X2, X2, X2...]
staticRX = reshape(repmat(cos(linspace(-pi/2, pi/2, staticLen)), staticDim, 1), [1, staticLen*staticDim])...
    *(Scale + 10) + Centre(1); % [X1, X1, X1, X2, X2, X2...]

staticTBY = reshape(repmat(sin(linspace(-pi/2, pi/2, staticLen)), staticDim, 1), [1, staticLen*staticDim])...
    .*(repmat(linspace(-staticRes, staticRes, staticDim), 1, staticLen) + Scale) + Centre(2); % [Y1, Y2, Y3, Y1, Y2, Y3...]
staticLY = reshape(repmat(sin(linspace(-pi/2, pi/2, staticLen)), staticDim, 1), [1, staticLen*staticDim])...
    *(Scale - 10) + Centre(2); % [Y1, Y1, Y1, Y2, Y2, Y2...]
staticRY = reshape(repmat(sin(linspace(-pi/2, pi/2, staticLen)), staticDim, 1), [1, staticLen*staticDim])...
    *(Scale + 10) + Centre(2); % [Y1, Y1, Y1, Y2, Y2, Y2...]

staticT = [staticTBX; staticTBY; staticTZ];
staticB = [staticTBX; staticTBY; staticBZ];
staticL = [staticLX; staticLY; staticLRZ];
staticR = [staticRX; staticRY; staticLRZ];

staticPoints = [];
if isStaticTop
    staticPoints = [staticPoints, staticT];
end
if isStaticBottom
    staticPoints = [staticPoints, staticB];
end
if isStaticLeft
    staticPoints = [staticPoints, staticL];
end
if isStaticRight
    staticPoints = [staticPoints, staticR];
end
% statisPoints = [staticB, staticT, staticL, staticR];
% if isStaticTop || isStaticBottom || isStaticLeft || isStaticRight
if ~isempty(staticPoints)
    environment.addStaticPoints(staticPoints);
end

% Dynamic Points
constantSE3ObjectMotion = [];

primitive1Dis = 5; % Distance to the sensor arc
primitive1InitialPose_rotm = eul2rot([(1/10)*pi, 0, 0]);
primitive1InitialPose_pos = [(Scale-primitive1Dis)*cos(-pi/2+pi/10); (Scale-primitive1Dis)*sin(-pi/2+pi/10); (1/10)*Ascend] + Centre;
primitive1InitialPose_SE3 = [primitive1InitialPose_rotm, primitive1InitialPose_pos; 0, 0, 0, 1];
primitive1Motion_rotm = eul2rot([(9/10)*pi/nSteps, 0, 0]);
primitive1Motion_SE3 = [primitive1Motion_rotm, [(9/10)*pi*(Scale-primitive1Dis)/nSteps; 0; (9/10)*Ascend/nSteps]; 0, 0, 0, 1];
primitive1Trajectory = ConstantMotionDiscretePoseTrajectory(t,primitive1InitialPose_SE3,primitive1Motion_SE3,'SE3');
environment.addEllipsoid([1 1 2.5],8,4,'R3',primitive1Trajectory); % Radii, Num of faces, Num of points, parameter type for GP points on surface, traj
constantSE3ObjectMotion(:,1) = primitive1Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));

primitive2Dis = 5; % Distance to the sensor arc
primitive2InitialPose_rotm = eye(3);
primitive2InitialPose_pos = [0; Scale+primitive2Dis; Ascend] + Centre;
primitive2InitialPose_SE3 = [primitive2InitialPose_rotm, primitive2InitialPose_pos; 0, 0, 0, 1];
primitive2Motion_rotm = eul2rot([-pi/nSteps, 0, 0]);
primitive2Motion_SE3 = [primitive2Motion_rotm, [pi*(Scale+primitive2Dis)/nSteps; 0; -Ascend/nSteps]; 0, 0, 0, 1];
primitive2Trajectory = ConstantMotionDiscretePoseTrajectory(t,primitive2InitialPose_SE3,primitive2Motion_SE3,'SE3');
environment.addEllipsoid([1 1 2.5],8,4,'R3',primitive2Trajectory); % Radii, Num of faces, Num of points, parameter type for GP points on surface, traj
constantSE3ObjectMotion(:,2) = primitive2Trajectory.RelativePoseGlobalFrameR3xso3(t(1),t(2));

% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

%% Check visibility
pointRelative = sensor.get('pointObservationRelative');
visibility = sensor.get('pointVisibility');
visibleRelative = pointRelative(visibility(:, 2)~=0, 2);

%% Plot Environment
figure(1)
viewPoint = [-35,35];
view(viewPoint)

% axisLimits = [-30,30,-5,30,-10,10];
% title('Environment')
axis equal
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
% axis(axisLimits)

hold on
grid on
primitive1Trajectory.plot(t,[0 0 0],'axesOFF')
primitive2Trajectory.plot(t,[0 0 0],'axesOFF')

cameraTrajectory.plot(t,[0 0 1],'axesOFF')
frames = sensor.plot(t,environment);

hold off

% print('VDO_Toy_Environment','-dpdf')
% implay(frames);

%% 5. Generate Measurements & Save to Graph File, load graph file as well
config.set('constantSE3Motion',constantSE3ObjectMotion);

datestring = string(datetime('now', 'Format', 'yyyyMMdd_HHmmSS'));
config.set('pointMotionMeasurement','point2DataAssociation');
status = mkdir(strcat(config.folderPath,config.sep,'Data',config.sep,config.graphFileFolderName,config.sep,datestring));

config.set('measurementsFileName', strcat(datestring, '/VDO_toy_measurements.graph'));
config.set('groundTruthFileName', strcat(datestring, '/VDO_toy_groundTruth.graph'));
sensor.generateMeasurements(config);
% writeDataAssociationVerticesEdges_constantSE3Motion(config,constantSE3ObjectMotion);


