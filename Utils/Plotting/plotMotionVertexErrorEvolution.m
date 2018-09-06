function plotMotionVertexErrorEvolution(solver, GTObjectMotion)

 nObjects = size(GTObjectMotion,2);

relativeTranslation = zeros(length(solver)-1,nObjects);
relativeRotation = zeros(length(solver)-1,nObjects);
f_deg_per_rad = 180/pi;

for i=1:length(solver)-1
    graph = solver(i).graphs(end);
    motionVertices = [graph.vertices(graph.identifyVertices('SE3Motion')).value];
    for j=1:size(motionVertices,2)
        objectMotion = motionVertices(:,j);
        objectGTMotion = GTObjectMotion(:,j);
        relativeMotion = AbsoluteToRelativePoseR3xso3(objectMotion,objectGTMotion);
        relativeTranslation(i,j) = norm(relativeMotion(1:3));
        relativeRotation(i,j) = wrapToPi(norm(relativeMotion(4:6)))*f_deg_per_rad;
    end 
end

%translation
figure;
for i=1:nObjects
    k = find(relativeTranslation(:,i)); 
    plot(k(1):length(solver)-1,relativeTranslation(k(1):end,i))    
    hold on
end
axis equal
legend('Object 1','Object 2','Object 3','Object 4','Object 5','Object 6',...
    'Object 7','Object 8','Object 9','Object 10','Object 11','Object 12');
xlabel('time step')
ylabel('Translation error (m)')
title({'Evolution of object motion error','translational component'})

%rotation
figure;
for i=1:nObjects
    k = find(relativeRotation(:,i)); 
    plot(k(1):length(solver)-1,relativeRotation(k(1):end,i))    
    hold on
end
axis equal
legend('Object 1','Object 2','Object 3','Object 4','Object 5','Object 6',...
    'Object 7','Object 8','Object 9','Object 10','Object 11','Object 12');
xlabel('time step')
ylabel('Rotation error (°)')
title({'Evolution of object motion error','rotational component'})
end