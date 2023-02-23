clear all 
% close all 

%% 1. Config
% time
nSteps = 256;
t0 = 1;
tN = 256;
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

config.set('cameraRelativePose', GP_Pose([0;0;0;arot(eul2rot([0, 0, -pi/2]))]));

%% Environment Parameter

Centre = [5; 10; 15]; % Centre (roughly) of the environment, where the traj sits
Scale = 200; % Roughly the half size of the Bounding Box of the environment, a radius

staticDim = 3;
staticLen = 9;
staticRes = 5;

%% Generate Environment

robotInitialPose_rotm = eye(3);
robotInitialPose_pos = [0; -Scale; 0] + Centre;
robotInitialPose_SE3 = [robotInitialPose_rotm, robotInitialPose_pos; 0, 0, 0, 1];
robotMotion_rotm = eul2rot([pi/nSteps, 0, 0]);
robotMotion_SE3 = [robotMotion_rotm, [pi*Scale/nSteps; 0; Scale/(4*nSteps)]; 0, 0, 0, 1];

robotTrajectory = ConstantMotionDiscretePoseTrajectory(t,robotInitialPose_SE3,robotMotion_SE3,'SE3');
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

environment = Environment();

staticX = repmat(linspace(1, staticRes*(staticDim-1)+1, staticDim), 1, staticDim) - 25;
staticY = 10*ones(1, staticDim^2);

staticTZ = reshape(repmat(linspace(0, Scale/4, staticLen), 3, 1), [1, staticLen*staticDim])... 
    + 10 + Centre(3); % [Z1, Z1, Z1, Z2, Z2, Z2...]
staticBZ = reshape(repmat(linspace(0, Scale/4, staticLen), 3, 1), [1, staticLen*staticDim])...
    - 10 + Centre(3); % [Z1, Z1, Z1, Z2, Z2, Z2...]
staticLRZ = repmat(linspace(-staticRes, staticRes, staticDim), 1, staticLen)...
    + reshape(repmat(linspace(0, Scale/4, staticLen), 3, 1), [1, staticLen*staticDim])...
    + Centre(3); % [Z1, Z2, Z3, Z1, Z2, Z3...]

staticTBX = reshape(repmat(cos(linspace(-pi/2, pi/2, staticLen)), 3, 1), [1, staticLen*staticDim])...
    .*(repmat(linspace(-staticRes, staticRes, staticDim), 1, staticLen) + Scale) + Centre(1); % [X1, X2, X3, X1, X2, X3...]
staticLX = reshape(repmat(cos(linspace(-pi/2, pi/2, staticLen)), 3, 1), [1, staticLen*staticDim])...
    *(Scale - 10) + Centre(1); % [X1, X1, X1, X2, X2, X2...]
staticRX = reshape(repmat(cos(linspace(-pi/2, pi/2, staticLen)), 3, 1), [1, staticLen*staticDim])...
    *(Scale + 10) + Centre(1); % [X1, X1, X1, X2, X2, X2...]

staticTBY = reshape(repmat(sin(linspace(-pi/2, pi/2, staticLen)), 3, 1), [1, staticLen*staticDim])...
    .*(repmat(linspace(-staticRes, staticRes, staticDim), 1, staticLen) + Scale) + Centre(2); % [Y1, Y2, Y3, Y1, Y2, Y3...]
staticLY = reshape(repmat(sin(linspace(-pi/2, pi/2, staticLen)), 3, 1), [1, staticLen*staticDim])...
    *(Scale - 10) + Centre(2); % [Y1, Y1, Y1, Y2, Y2, Y2...]
staticRY = reshape(repmat(sin(linspace(-pi/2, pi/2, staticLen)), 3, 1), [1, staticLen*staticDim])...
    *(Scale + 10) + Centre(2); % [Y1, Y1, Y1, Y2, Y2, Y2...]

staticT = [staticTBX; staticTBY; staticTZ];
staticB = [staticTBX; staticTBY; staticBZ];
staticL = [staticLX; staticLY; staticLRZ];
staticR = [staticRX; staticRY; staticLRZ];

statisPoints = [staticB, staticT, staticL, staticR];
environment.addStaticPoints(statisPoints);



% occlusion sensor
sensor = SimulatedEnvironmentOcclusionSensor();
sensor.addEnvironment(environment);
sensor.addCamera(config.fieldOfView,cameraTrajectory);
sensor.setVisibility(config,environment);

%% 4. Plot Environment
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
cameraTrajectory.plot(t,[0 0 1],'axesOFF')
frames = sensor.plot(t,environment);

hold off

% print('VDO_Toy_Environment','-dpdf')
% implay(frames);




