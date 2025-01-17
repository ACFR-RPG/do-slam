%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 11/01/18
% Contributors:
%--------------------------------------------------------------------------
% realData_Calibration

%% general setup
clc;
clear all;
close all;

%% config setup
config = CameraConfig();
config = setUnitTestConfig(config);
config.set('groundTruthFileName','graphFileGT_calibration.graph');
config.set('measurementsFileName','graphFile_calibration.graph');
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.05,0.05,0.05]');

%% w/o SE3
config.set('pointMotionMeasurement','Off')
measurementsFilePath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.measurementsFileName);
measurementsFilePath2 = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.measurementsFileName,'2');
copyfile(measurementsFilePath,measurementsFilePath2);
deleteDataAssociationFromGraphFile(measurementsFilePath2)
config.set('measurementsFileName','graphFile_calibration.graph2');
groundTruthNoSE3Cell = graphFileToCell(config,config.groundTruthFileName);
measurementsNoSE3Cell = graphFileToCell(config,config.measurementsFileName);
%% solve
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
initialGraphN.saveGraphFile(config,'calibration_resultsNoSE3.graph');

%% w/ SE3
config.set('pointMotionMeasurement','point2DataAssociation');
filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,'calibration_resultsNoSE3.graph');
filepath2 = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,config.groundTruthFileName,'2');
copyfile(filepath,filepath2);
deleteEdgesFromGraphFile(filepath2);
% augment graphFile with 2POINTS_DataAssociation
fid = fopen(measurementsFilePath, 'r');
Data = textscan(fid,'%s','Delimiter','\n');
CStr = Data{1};
fclose(fid);
IndexC = strfind(CStr, '2POINTS_DataAssociation');
Index = find(~cellfun('isempty', IndexC));
fid = fopen(filepath2, 'a');
for i=1:size(Index,1)
fprintf(fid,'%s ',string(CStr(Index(i))));
fprintf(fid,'\n');
end
fclose(fid);
%% get constantSE3ObjectMotion from solution of NoSE3
constantSE3ObjectMotion = [0.1;0.1;0.1;0.001;0.001;0.001];
config.set('groundTruthFileName','graphFileGT_calibration.graph2');
writeDataAssociationVerticesEdges(config,constantSE3ObjectMotion);
measurementsCell = graphFileToCell(config,config.measurementsFileName);
groundTruthCell  = graphFileToCell(config,config.groundTruthFileName);
%% solve
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
graphN.saveGraphFile(config,'calibration_results.graph');

%% plot graph files
% h = figure;
axis equal;
xlabel('x')
ylabel('y')
zlabel('z')
hold on
plotGraph(config,graphN,[1 0 0]);

figure
subplot(2,2,1)
spy(solverEnd.systems(end).A)
subplot(2,2,2)
spy(solverEnd.systems(end).H)
subplot(2,2,3)
spy(solverEnd.systems(end).covariance)
subplot(2,2,4)
spy(solverEnd.systems(end).covSqrtInv)