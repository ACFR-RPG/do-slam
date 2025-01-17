

for i = 0:20
    displayOn = 0;
    folder = '/media/mina/ACRV Samsung SSD T5/KITTI dataset/tracking/data_tracking_oxts/training/oxts/';
    filename = strcat(repmat('0',1,4-numel(num2str(i))),num2str(i),'.txt');
    
    GPSIMUFile = strcat(folder,filename);
    GPSIMUFileID = fopen(GPSIMUFile,'r');
    GPSIMUData = textscan(GPSIMUFileID,'%s','delimiter','\n','whitespace',' ');
    GPSIMUCStr = GPSIMUData{1};
    fclose(GPSIMUFileID);
    
    fileToWrite = fopen(strcat('/media/mina/ACRV Samsung SSD T5/KITTI dataset/tracking/extrinsics/',filename),'w');
    
    for j = 1:size(GPSIMUCStr,1)
        
        oxts = str2double(strsplit(GPSIMUCStr{j},' '));
        
        if j==1
           scale = cos(oxts(1,1) * pi / 180.0);
        end
        
        % translation vector
        [t(1,1) t(2,1)] = latlonToMercator(oxts(1,1),oxts(1,2),scale);
        t(3,1) = oxts(1,3);
        
        % rotation matrix (OXTS RT3000 user manual, page 71/92)
        rx = oxts(1,4); % roll
        ry = oxts(1,5); % pitch
        rz = oxts(1,6); % heading
        Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)]; % base => nav  (level oxts => rotated oxts)
        Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)]; % base => nav  (level oxts => rotated oxts)
        Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1]; % base => nav  (level oxts => rotated oxts)
        R  = Rz*Ry*Rx;
        
        % normalize translation and rotation (start at 0/0/0)
        if j == 1
            Tr_0_inv = inv([R t;0 0 0 1]);
        end
        
        % add pose
        pose{j} = Tr_0_inv*[R t;0 0 0 1];
        
        poseMatrix = Tr_0_inv*[R t;0 0 0 1];
        
        
        velo_to_cam = [7.533745000000e-03 -9.999714000000e-01 -6.166020000000e-04 -4.069766000000e-03;
            1.480249000000e-02 7.280733000000e-04 -9.998902000000e-01 -7.631618000000e-02;
            9.998621000000e-01 7.523790000000e-03 1.480755000000e-02 -2.717806000000e-01;
            0 0 0 1];
        
        imu_to_velo = [9.999976000000e-01 7.553071000000e-04 -2.035826000000e-03 -8.086759000000e-01;
            -7.854027000000e-04 9.998898000000e-01 -1.482298000000e-02 3.195559000000e-01;
            2.024406000000e-03 1.482454000000e-02 9.998881000000e-01 -7.997231000000e-01;
            0 0 0 1];
        
        Tr = velo_to_cam*imu_to_velo;
        poseMatrix = Tr*poseMatrix*inv(Tr);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         if j>1
%             if j == 2
%                 currentPose = poseMatrix;
%                 vec = currentPose(1:3,4);
%                 ang = atan2(norm(cross(vec',[1 0 0])),dot(vec',[1 0 0]));
%                 Tr = [cos(ang) -sin(ang) 0 0; sin(ang) cos(ang) 0 0; 0 0 1 0; 0 0 0 1]
%             end
%         poseMatrix = Tr'*poseMatrix*Tr;
%         pose{j} = poseMatrix;
%         end
%
%             % velocity
%             if j>1
%                 if j == 2
%                     lastPose = eye(4);
%                 end
%                 currentPose = poseMatrix;
%                 tr = currentPose(1:3,4) - lastPose(1:3,4);
%                 Rot = lastPose(1:3,1:3);
%                 v = Rot'*tr;
%                 plot3([0 v(1)], [0 v(2)], [0 v(3)]);
%                 hold on;
%                 lastPose = currentPose;
%             end
%             xlabel('x');
%             ylabel('y');
%             zlabel('z');
%             axis equal
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        line = reshape(poseMatrix', 1, 16);
        if j == 1
            fprintf(fileToWrite,'%s %s %s %s %s %s %s %s %s %s %s %s %s %d %d %d %d\n',...
            'frame','r1,1','r1,2','r1,3','t1','r2,1','r2,2','r2,3','t2','r3,1','r3,2','r3,3','t3',0,0,0,1);
        end
        fprintf(fileToWrite,'%d %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f %0.9f\n',...
            j-1,line(1),line(2),line(3),line(4),line(5),line(6),line(7),line(8),line(9),line(10),line(11),line(12),...
            line(13),line(14),line(15),line(16));
    end
    
    % plot every pose
    if displayOn
        figure; hold on; axis equal;
        l = 3; % coordinate axis length
        A = [0 0 0 1; l 0 0 1; 0 0 0 1; 0 l 0 1; 0 0 0 1; 0 0 l 1]';
        for j=1:5:length(pose)
            B = pose{j}*A;
            plot3(B(1,1:2),B(2,1:2),B(3,1:2),'-r','LineWidth',2); % x: red
            plot3(B(1,3:4),B(2,3:4),B(3,3:4),'-g','LineWidth',2); % y: green
            plot3(B(1,5:6),B(2,5:6),B(3,5:6),'-b','LineWidth',2); % z: blue
        end
        xlabel('x');
        ylabel('y');
        zlabel('z');
    end
    clear;
end


