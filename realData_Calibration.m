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
% config.set('measurementsFileName','graphFile_CalibrationGT.graph');
config.set('measurementsFileName','graphFile_Calibration.graph');
config.set('motionModel','constantSE3MotionDA');
config.set('std2PointsSE3Motion', [0.1,0.1,0.1]');
config = setUnitTestConfig(config);

%% solve
% writeDataAssociationVerticesEdges
% create a GT graph File to lock the first pose
groundTruthCell = graphFileToCell(config,config.groundTruthFileName);
measurementsCell = graphFileToCell(config,config.measurementsFileName);

timeStart = tic;
graph0 = Graph();
solver = graph0.process(config,measurementsCell,groundTruthCell);
solverEnd = solver(end);
totalTime = toc(timeStart);
fprintf('\nTotal time solving: %f\n',totalTime)

graphN  = solverEnd.graphs(end);
graphN.saveGraphFile(config,'results_Calibration.graph');
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