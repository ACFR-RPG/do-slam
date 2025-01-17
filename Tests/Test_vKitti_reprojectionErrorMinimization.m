%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 30/11/2018
% Contributors:
%--------------------------------------------------------------------------
% Test static only vKITTI reprojectioin error minimization
%--------------------------------------------------------------------------
% vKitti main
rng(12)
config = CameraConfig();
K = [725,   0,     620.5;
    0,    725,     187.0;
    0,      0,        1];
config.set('focalLength',725);
config.set('opticalCentreX',620.5);
config.set('opticalCentreY',187.0);
% config.set('R',[0,0,1,0; -1,0,0,0; 0,-1,0,0;0,0,0,1]);
config.set('R',eye(4));
config = setAppConfig(config);
config.absoluteToRelativePointHandle = @AbsoluteToRelativePositionR3xso3Image;
config.set('landmarkErrorToMinimize' ,'reprojectionKnownIntrinsics');
config.set('noiseModel','Off');
config.set('stdPosePixel',[2;2]);
% config.set('mode','initialisation');


%% 5. Generate Measurements & Save to Graph File, load graph file as well
config.set('pointMotionMeasurement','Off')
% config.set('measurementsFileName','finalNoiseSequence0001_short_reprojection_MeasStaticOnly_noNoise.graph')
% config.set('groundTruthFileName','finalNoiseSequence0001_short_reprojection_GTStaticOnly_noNoise.graph')

config.set('measurementsFileName','Sequence0001_380to383_noR_noNoise_Meas.graph')
config.set('groundTruthFileName','Sequence0001_380to383_noR_noNoise_GT.graph')


writeGraphFileImagePlane(config)
config.set('posePointEdgeLabel','EDGE_2D_PIXEL');
config.set('pointInitialisation','3DMeasurement');
config.set('measurementsFileName',strcat(config.measurementsFileName(1:end-6),'_Test.graph'))
clc
groundTruthCellReprojectionStaticOnly = graphFileToCell(config,config.groundTruthFileName);
measurementsCellReprojectionStaticOnly = graphFileToCell(config,config.measurementsFileName);
 
%% 6. Solve
%% 6.1 reprojection error minimization
timeStartReprojection = tic;
reprojectionGraph0 = Graph();
reprojectionSolver = reprojectionGraph0.process(config,measurementsCellReprojectionStaticOnly,groundTruthCellReprojectionStaticOnly);
reprojectionSolverEnd = reprojectionSolver(end);
totalTimeReprojection = toc(timeStartReprojection);
%get desired graphs & systems
reprojectionGraph0  = reprojectionSolverEnd.graphs(1);
reprojectionGraphN  = reprojectionSolverEnd.graphs(end);
%save results to graph file
reprojectionGraphN.saveGraphFile(config,'finalNoiseSequence0001_short_reprojection_resultsStaticOnly.graph');

%% 6.2 3D error minimization
config.set('landmarkErrorToMinimize' ,'3D');
config.set('posePointEdgeLabel','EDGE_3D');
config.absoluteToRelativePointHandle = @AbsoluteToRelativePositionR3xso3;
% config.set('measurementsFileName','finalNoiseSequence0001_short_reprojection_MeasStaticOnly_noNoise.graph')
% config.set('groundTruthFileName','finalNoiseSequence0001_short_reprojection_GTStaticOnly_noNoise.graph')
config.set('measurementsFileName','Sequence0001_380to383_noR_noNoise_Meas.graph')
config.set('groundTruthFileName','Sequence0001_380to383_noR_noNoise_GT.graph')
clc
groundTruthCellStaticOnly = graphFileToCell(config,config.groundTruthFileName);
measurementsCellStaticOnly = graphFileToCell(config,config.measurementsFileName);
timeStart = tic;
Graph0 = Graph();
Solver = Graph0.process(config,measurementsCellStaticOnly,groundTruthCellStaticOnly);
SolverEnd = Solver(end);
totalTime = toc(timeStart);
%get desired graphs & systems
Graph0  = SolverEnd.graphs(1);
GraphN  = SolverEnd.graphs(end);
%save results to graph file
GraphN.saveGraphFile(config,'finalNoiseSequence0001_short_3D_resultsStaticOnly.graph');

%% 7. Error analysis
fprintf('\nTotal time solving (reprojection error minimization): %f\n',totalTimeReprojection)
fprintf('\nTotal time solving (3D error minimization): %f\n',totalTime)

%load ground truth into graph, sort if required
graphGTReprojection = Graph(config,groundTruthCellReprojectionStaticOnly);
fprintf('\nReprojection error minimization results:\n')
resultsReprojection = errorAnalysis(config,graphGTReprojection,reprojectionGraphN);
fprintf('\nReprojection error:\n')
reprojectionError = calculate_reprojection_error(config,graphGTReprojection,reprojectionGraphN);
% 
graphGT = Graph(config,groundTruthCellStaticOnly);
fprintf('\n3D error minimization results:\n')
results3D = errorAnalysis(config,graphGT,GraphN);

fprintf('\nReprojection error:\n')
reprojectionError = calculate_reprojection_error(config,graphGT,GraphN);