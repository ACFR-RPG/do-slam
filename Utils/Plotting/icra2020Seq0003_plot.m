measFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0003-0-40_Meas.graph';
resultFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0003-0-40_result.graph';
fileID = fopen(resultFilePath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
resultCStr = Data{1};
fclose(fileID);

resultPoses = [];
resultPoints = [];
resultPointIds = [];
resultMotions = [];
for i = 1:size(resultCStr,1)
    line = strsplit(resultCStr{i},' ');
    if strcmp(line{1},'VERTEX_POSE_R3_SO3')
        resultPoses = [resultPoses, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    elseif strcmp(line{1},'VERTEX_POINT_3D')
        resultPoints = [resultPoints, [str2double(line{3}); str2double(line{4}); str2double(line{5})]];
        resultPointIds = [resultPointIds, str2double(line{2})];
    elseif strcmp(line{1},'VERTEX_SE3Motion')
        resultMotions = [resultMotions, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    end
end

colors = {'magenta','radioactive green','leather','blue','red','black','cornflower',...
    'sapphire','swamp','plum','light bluish green','butterscotch','cinnamon','chartreuse','green'}; 

allDynamicPointIds = identifyDynamicPointIndices(measFilePath);
staticPointIds = setdiff(resultPointIds, allDynamicPointIds);

objectPoints = vKITTI_pointObservability(measFilePath);

clear h
figure; hold on; axis equal;
% l = 1; % coordinate axis length
% A = [0 0 0 1; l 0 0 1; 0 0 0 1; 0 l 0 1; 0 0 0 1; 0 0 l 1]';
% for i=1:1:size(resultPoses,2)
%   B = poseToTransformationMatrix(resultPoses(:,i))*A;
%   plot3(B(1,1:2),B(2,1:2),B(3,1:2),'-r','LineWidth',2); % x: red
%   plot3(B(1,3:4),B(2,3:4),B(3,3:4),'-g','LineWidth',2); % y: green
%   plot3(B(1,5:6),B(2,5:6),B(3,5:6),'-b','LineWidth',2); % z: blue
% end
for i=1:1:size(resultPoses,2)
    iPose = resultPoses(:,i);
    plotiCamera = plotCamera('Location',iPose(1:3),'Orientation',rot(-iPose(4:6))); %LHS invert pose
    plotiCamera.Opacity = 0.1;
    plotiCamera.Size = 0.5;
    plotiCamera.Color = 'red';
end

% for i=1:size(resultPoints,2)
%     for j = 1:size(objectPoints,1)     
%         iObjectPoints = [objectPoints{j,:}];
%         if ismember(resultPointIds(i),iObjectPoints)
%             if ismember(j,[1:26,28])
%                 scatter3(resultPoints(1,i),resultPoints(2,i),resultPoints(3,i),7,...
%                     'MarkerEdgeColor',rgb(colors{1}),'MarkerFaceColor',rgb(colors{1}));
%             elseif ismember(j,[27,29:42])
%                 scatter3(resultPoints(1,i),resultPoints(2,i),resultPoints(3,i),7,...
%                     'MarkerEdgeColor',rgb(colors{2}),'MarkerFaceColor',rgb(colors{2}));
%             end
%         end
%     end
% end


staticPoints = zeros(3,length(staticPointIds));
n = 0;
for i=1:size(resultPoints,2)
    if ismember(resultPointIds(i),staticPointIds)
        n = n+1;
        staticPoints(:,n) =  resultPoints(:,i);
    end
end

plotStaticStructure = plot3(staticPoints(1,:),staticPoints(2,:),staticPoints(3,:),...
    'Color','b','Marker','.','MarkerSize',7,'LineStyle','none');
h(1) = plotStaticStructure;

for j = 1:size(objectPoints,1)
    for k = 1:size(objectPoints,2)
        ids = [objectPoints{j,k}];
        if ~isempty(ids)
            Points = [];
            for m = 1:length(ids)
                Points = [Points, resultPoints(:,resultPointIds == ids(m))];
            end
            if ismember(j,[1:26,28])
                plotObject1 = plot3(Points(1,:),Points(2,:),Points(3,:),'.',...
                    'MarkerSize',7, 'Color', rgb(colors{1}),'LineStyle','none');
                h(2) = plotObject1;
                hold on
            else
                plotObject2 = plot3(Points(1,:),Points(2,:),Points(3,:),'.',...
                    'MarkerSize',7, 'Color', rgb(colors{2}),'LineStyle','none');
                h(3) = plotObject2;
                hold on
            end
        end
    end
end


xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis equal
xlim([-5 10])
zlim([30 40])
view(0,0)
legend(h, {' static structure ',' object 1 ',' object 2 '})


AxesH    = gca;
UpVector = [-sind(30), cosd(30), 0];
DAR      = get(AxesH, 'DataAspectRatio');
AxesH.Box = 'on';
AxesH.FontSize = 16;
set(AxesH, 'CameraUpVector', DAR .* UpVector);
 

