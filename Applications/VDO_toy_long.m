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
Scale = 20; % Roughly the half size of the Bounding Box of the environment, a radius

%% Generate Environment

% robotInitialPose_R3xso3 = [-Scale/2 + Centre(1); -Scale/2 + Centre(2); -Scale/4 + Centre(3); 0; 0; pi/2];
% robotMotion_R3xso3 = [pi*Scale/nSteps; 0; Scale/(2*nSteps); arot(eul2rot([pi/nSteps,0,0]))];
robotInitialPose_R3xso3 = [10; 20; 30; 0; 0; 0];
robotMotion_R3xso3 = [1/nSteps; 0; 0; 0; 0; 0];


robotTrajectory = ConstantMotionDiscretePoseTrajectory(t,robotInitialPose_R3xso3,robotMotion_R3xso3,'R3xso3');
cameraTrajectory = RelativePoseTrajectory(robotTrajectory,config.cameraRelativePose);

environment = Environment();

staticDim = 7;
staticRes = 5;
staticX = repmat(linspace(1, staticRes*(staticDim-1)+1, staticDim), 1, staticDim) - 25;
staticZ = reshape(repmat(linspace(1, staticRes*(staticDim-1)+1, 7), staticDim, 1), 1, []) - 15;
staticY = 10*ones(1, staticDim^2);
statisPoints = [staticX; staticY; staticZ];
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




