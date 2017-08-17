function config = setUnitTestConfig(config)
%SETUNITTESTCONFIG - makes it easier to write common configs for all the
%unit tests, required as new solver features (such as edges) are added.

% set properties of Config
config.set('t',0.1);
config.set('rngSeed',1);
config.set('noiseModel','Gaussian');
% config.set('noiseModel','Off');
config.set('poseParameterisation','R3xso3');

% temporarily changed function handles to public for setting

config.absoluteToRelativePoseHandle = @AbsoluteToRelativePoseR3xso3;
config.relativeToAbsolutePoseHandle = @RelativeToAbsolutePoseR3xso3;
if strcmp(config.motionModel,'constantSE3Rob') ||...
        strcmp(config.motionModel,'constantSE3Mina')||...
        strcmp(config.motionModel,'constantSE3')
    config.absoluteToRelativePointHandle = @AbsoluteToRelativePositionR3xso3Normalised;
    config.relativeToAbsolutePointHandle = @RelativeToAbsolutePositionR3xso3Normalised;
else
    config.absoluteToRelativePointHandle = @AbsoluteToRelativePositionR3xso3;
    config.relativeToAbsolutePointHandle = @RelativeToAbsolutePositionR3xso3;    
end
    
config.set('cameraPointParameterisation','euclidean');
config.set('cameraControlInput','relativePose');
config.set('poseVertexLabel'     ,'VERTEX_POSE_R3_SO3');
config.set('pointVertexLabel'    ,'VERTEX_POINT_3D');
config.set('planeVertexLabel'    ,'VERTEX_PLANE_4D');
config.set('posePoseEdgeLabel'   ,'EDGE_R3_SO3');
config.set('posePointEdgeLabel'  ,'EDGE_3D');
config.set('pointPlaneEdgeLabel' ,'EDGE_1D');
config.set('pointPointEdgeLabel' ,'EDGE_2POINTS');
config.set('pointPointEdgeSE3Label','EDGE_2POINTS_SE3')
config.set('point3EdgeLabel','EDGE_3POINTS')
config.set('velocityVertexLabel','VERTEX_VELOCITY')
config.set('pointVelocityEdgeLabel','EDGE_2POINTS_VELOCITY')
config.set('SE3MotionVertexLabel','VERTEX_SE3Motion')
config.set('pointSE3MotionEdgeLabel','EDGE_2POINTS_SE3Motion')
config.set('posePriorEdgeLabel','EDGE_6D');
config.set('graphFileFolderName' ,'GraphFiles');

% set pose prior
orientation = [pi/180;pi/180;pi/180]; % 1 degree position error
config.set('stdPosePrior',[0.005;0.005;0.005;orientation]);

% set point prior error
if strcmp(config.motionModel,'constantSE3Rob') ||...
        strcmp(config.motionModel,'constantSE3Mina')||...
        strcmp(config.motionModel,'constantSE3')
    config.set('stdPointPrior',[0.01,0.01,0.01,0.01]');
else 
    config.set('stdPointPrior',[0.01,0.01,0.01]');
end

% set odometry error
orientation = [pi/36;pi/36;pi/36]; % 0.5 degree position error
config.set('stdPosePose'  ,[0.4,0.4,0.4,orientation']');
if strcmp(config.motionModel,'constantSE3Rob') ||...
        strcmp(config.motionModel,'constantSE3Mina')||...
        strcmp(config.motionModel,'constantSE3')
    config.set('stdPosePoint' ,[0.4,0.4,0.4,0.01]');
else 
    config.set('stdPosePoint' ,[0.4,0.4,0.4]');
end


config.set('stdPointPlane',0.001);
% set properties of CameraConfig
config.set('fieldOfView',[-pi/3,pi/3,-pi/6,pi/6,1,10]); %az,el,r limits
config.set('cameraRelativePose',GP_Pose([0,0,0,0,0,-pi/8]'));
% set properties of solverConfig
%   dimensions
config.set('dimPose',6);
if strcmp(config.motionModel,'constantSE3Rob') ||...
        strcmp(config.motionModel,'constantSE3Mina') ||...
        strcmp(config.motionModel,'constantSE3') 
    config.set('dimPoint',4);
else 
    config.set('dimPoint',3);
end
%   plane parameterisation'
config.set('planeNormalParameterisation','S2');
%   constraints
config.set('applyAngleConstraints',0);
config.set('automaticAngleConstraints',0);
%   first linearisation point
config.set('startPose','initial');
%   static assumption
config.set('staticAssumption',0);
%   solver settings
config.set('sortVertices',0);
config.set('sortEdges',0);
config.set('processing','batch');
config.set('nVerticesThreshold',100);
config.set('nEdgesThreshold',200);
config.set('solveRate',5);
config.set('solverType','Levenberg-Marquardt');
config.set('threshold',10e-4);
config.set('maxNormDX',1e10);
config.set('maxIterations',1000);
config.set('displayProgress',1);
config.set('savePath',pwd);
config.set('plotPlanes',0);


end
