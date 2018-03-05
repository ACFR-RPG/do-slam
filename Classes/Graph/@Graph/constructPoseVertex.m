function [obj] = constructPoseVertex(obj,config,edgeRow)
%CONSTRUCTPOSEVERTEX Constructs vertex representing a pose. Pose is
%estimated from previous pose and odometry measurement

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
inputVertices = edgeRow{3};
outputVertices = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. compute pose
switch edgeLabel
    case config.posePriorEdgeLabel %prior
        pose2 = edgeValue;
    case config.posePoseEdgeLabel %odometry
        pose1 = obj.vertices(inputVertices).value;
        controlInput = edgeValue;
        switch config.cameraControlInput
            case 'relativePose'
                pose2 = config.relativeToAbsolutePoseHandle(pose1,controlInput);
                %                 pose2(4:6) = wrapAxisAngle(pose2(4:6),'pi');
                %                 pose2 = RelativeToAbsolutePose(pose1,controlInput);
                %                 pose2 = Relative2AbsoluteSE3(pose1,controlInput);
            case 'relativeVelocity'
                pose2 = config.relativeToAbsolutePoseHandle(pose1,controlInput*config.dt);
                %                 pose2(4:6) = wrapAxisAngle(pose2(4:6),'pi');
                %                 pose2 = RelativeToAbsolutePose(pose1,controlInput*config.dt);
                %                 pose2 = Relative2AbsoluteSE3(pose1,controlInput*config.dt);
        end
    case config.posePointEdgeLabel %to initialise a pose from point measurement - multicamera system
%         pose2 = zeros(6,1);
        if inputVertices == 517
            pose2 = [0;3;0;0;0;1.57];
            disp('camera 2 initialized')
            if ~strcmp(config.noiseModel,'Off')
                for i = 1:size(pose2,2)
                    pose2(:,i) = config.relativeToAbsolutePoseHandle(pose2(:,i),config.stdPosePose(:,i));
                end
            end
        elseif inputVertices == 1140
            disp('camera 3 initialized')
            pose2 = [-3;0;0;0;0;3.14];
            if ~strcmp(config.noiseModel,'Off')
                for i = 1:size(pose2,2)
                    pose2(:,i) = config.relativeToAbsolutePoseHandle(pose2(:,i),config.stdPosePose(:,i));
                end
            end
        elseif inputVertices == 241
            disp('camera 2 initialized')
%             pose2 = [1.8;0;0;0;0;-1.2];
            pose2 = [0;3;0;0;0;1.57];
            if ~strcmp(config.noiseModel,'Off')
                for i = 1:size(pose2,2)
                    pose2(:,i) = config.relativeToAbsolutePoseHandle(pose2(:,i),config.stdPosePose(:,i));
                end
            end
        elseif inputVertices == 757 %984
            disp('camera 3 initialized')
%             pose2 = [1.8;4.7;0;0;0;1.2];
            pose2 = [-3;0;0;0;0;3.14];
            if ~strcmp(config.noiseModel,'Off')
                for i = 1:size(pose2,2)
                    pose2(:,i) = config.relativeToAbsolutePoseHandle(pose2(:,i),config.stdPosePose(:,i));
                end
            end
        elseif inputVertices == 1380 %1278
            disp('camera 4 initialized')
%             pose2 = [0;4.7;0;0;0;1.93];
              pose2 = [0;-3;0;0;0;-1.57];
%             if ~strcmp(config.noiseModel,'Off')
%                 for i = 1:size(pose2,2)
%                     pose2(:,i) = config.relativeToAbsolutePoseHandle(pose2(:,i),config.stdPosePose(:,i));
%                 end
%             end
        end
        outputVertices = inputVertices;
    otherwise; error('error, wrong edgeLabel')
end

%% 3. vertex properties
value = pose2;
covariance = []; %not using this property yet
type = 'pose';
iEdges = [edgeIndex];
index = outputVertices;

%% 4. construct vertex
obj.vertices(index) = Vertex(value,covariance,type,iEdges,index);

end

