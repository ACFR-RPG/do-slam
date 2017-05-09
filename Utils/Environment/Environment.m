classdef Environment < ArrayGetSet
    %Environment class stores environment primitives and environment points
    %and has methods to construct and manipulate them
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        environmentPrimitives
        environmentPoints
    end
    
    properties(Dependent)
        nEnvironmentPrimitives
        nEnvironmentPoints
    end
    
    %% 2. Methods
    % Dependent properties
    methods
        function nEnvironmentPrimitives = get.nEnvironmentPrimitives(self)
            nEnvironmentPrimitives = numel(self.environmentPrimitives);
        end
        
        function nEnvironmentPoints = get.nEnvironmentPoints(self)
            nEnvironmentPoints = numel(self.environmentPoints);
        end
    end   
    
    % Constructor
    methods(Access = public)
        function self = Environment()
        end
    end
    
    % Get & Set
    methods(Access = public)
        function out = getSwitch(self,property,varargin)
        	out = self.(property);
        end
        
        function self = setSwitch(self,property,value,varargin)
        	self.(property) = value;
        end
        
    end
    
    % Construct primitives
    methods(Access = public)
        % Default
        function self = addPrimitive(self,positionsRelative,parameterisation,trajectory)
            nPoints = size(positionsRelative,2);
            
            %construct fixedPoints
            fixedPoints(nPoints) = FixedEnvironmentPoint();
            for i = 1:nPoints
                fixedPoints(i) = FixedEnvironmentPoint();
                fixedPoints(i).set('trajectory',StaticPointTrajectory(positionsRelative(:,i),parameterisation));
            end
            
            %construct primitive
            primitive = EP_Default();
            primitive.set('trajectory',trajectory);
            
            %pair
            self.addPrimitiveAndPoints(primitive,fixedPoints);
        end
        
        % Rectangle
        function self = addRectangle(self,sideLengths,nPoints,distribution,rectangleTrajectory)
            
            %relative positions
            switch distribution
                case 'uniform'
                    rectanglePositionsRelative = generateRectanglePoints(sideLengths,nPoints,'uniform');
                case 'edges'
                    rectanglePositionsRelative = generateRectanglePoints(sideLengths,nPoints,'edges');
                case 'mixed'
                    nCentrePoints = 0.7*nPoints;
                    nEdgePoints   = nPoints - nCentrePoints;
                    rectanglePositionsRelative = [generateRectanglePoints(sideLengths,nCentrePoints,'uniform'),...
                                                  generateRectanglePoints(sideLengths,nEdgePoints,'edges')];
            end
            
            %initialise fixed points
            rectangleFixedPoints(nPoints) = FixedEnvironmentPoint();
            for i = 1:nPoints
                rectangleFixedPoints(i) = FixedEnvironmentPoint();
                rectangleFixedPoints(i).set('trajectory',StaticPointTrajectory(rectanglePositionsRelative(:,i),'R3'));
            end
            
            %initialise EP_Rectangle primitive
            rectanglePrimitive = EP_Rectangle(sideLengths,rectangleTrajectory);
            
            %pair
            self.addPrimitiveAndPoints(rectanglePrimitive,rectangleFixedPoints);
            
        end
       
    end
    
    % Add primitive & points
    methods(Access = private, Hidden = true)
        function self = addPrimitiveAndPoints(self,primitive,fixedPoints)
            primitiveIndex = self.nEnvironmentPrimitives + 1;
            pointIndexes   = self.nEnvironmentPoints + 1:self.nEnvironmentPoints + numel(fixedPoints);
            
            %set primitive indexes
            primitive.set('index',primitiveIndex);
            primitive.set('pointIndexes',pointIndexes);
            
            %set point indexes
            fixedPoints.set('index',pointIndexes);
            fixedPoints.set('primitiveIndexes',repmat(primitiveIndex,1,numel(fixedPoints)));
            fixedPoints.set('referencePrimitiveIndex',repmat(primitiveIndex,1,numel(fixedPoints)));
            
            %add to self
            self.environmentPrimitives = [self.environmentPrimitives primitive];
            self.environmentPoints = [self.environmentPoints fixedPoints];
        end
    end

    % plot
    methods(Access = public)
        function plot(self,t)
            %identify dynamic points and dynamic primitives
            dynamicPointIndexes = [];
            dynamicPrimitiveIndexes = [];
            for i = 1:self.nEnvironmentPrimitives
                if ~strcmp('StaticPoseTrajectory',class(self.environmentPrimitives(i).get('trajectory')))
                    dynamicPointIndexes = [dynamicPointIndexes self.environmentPrimitives(i).get('pointIndexes')];
                    dynamicPrimitiveIndexes = [dynamicPrimitiveIndexes self.environmentPrimitives(i).get('index')];
                end
            end
            for i = 1:self.nEnvironmentPoints
                if ~strcmp('StaticPointTrajectory',class(self.environmentPoints(i).get('trajectory')))
                     dynamicPointIndexes = [dynamicPointIndexes self.environmentPoints(i).get('index')];
                end
            end
            staticPointIndexes = setdiff(1:self.nEnvironmentPoints,dynamicPointIndexes);
            staticPrimitiveIndexes = setdiff(1:self.nEnvironmentPrimitives,dynamicPrimitiveIndexes);
            self.plotStatic(staticPointIndexes,staticPrimitiveIndexes)
            self.plotDynamic(dynamicPointIndexes,dynamicPrimitiveIndexes,t)
        end
    end
    
    methods(Access = private, Hidden = true)
        function plotStatic(self,staticPointIndexes,staticPrimitiveIndexes)
            nPoints     = numel(staticPointIndexes);
            nPrimitives = numel(staticPrimitiveIndexes);
            %get point positions
            positions = zeros(3,nPoints);
            for i = 1:nPoints
                if strcmp('FixedEnvironmentPoint',class(self.environmentPoints(staticPointIndexes(i))))
                    pointTrajectory = self.environmentPoints(staticPointIndexes(i)).get('trajectory');
                    primitiveIndex = self.environmentPoints(staticPointIndexes(i)).get('referencePrimitiveIndex');
                    primitiveTrajectory = self.environmentPrimitives(primitiveIndex).get('trajectory');
                    positions(:,i) = pointTrajectory.RelativeToAbsolutePoint(primitiveTrajectory,0,'R3Position');
                else
                    positions(:,i) = self.environmentPoints(staticPointIndexes(i)).get('R3Position',0);
                end
            end
            %plot positions
            plot3(positions(1,:),positions(2,:),positions(3,:),'k.')
            %plot primitives
%             for i = 1:staticPrimitiveIndexes
%                 self.environmentPrimitives(staticPrimitiveIndexes(i)).plot()
%             end
        end
        function plotDynamic(self,dynamicPointIndexes,dynamicPrimitiveIndexes,t)
            nSteps = numel(t);
            for i = 1:nSteps
                nPoints     = numel(dynamicPointIndexes);
                nPrimitives = numel(dynamicPrimitiveIndexes);
                %get point positions
                positions = zeros(3,nPoints);
                for j = 1:nPoints
                    if strcmp('FixedEnvironmentPoint',class(self.environmentPoints(dynamicPointIndexes(j))))
                        pointTrajectory = self.environmentPoints(dynamicPointIndexes(j)).get('trajectory');
                        primitiveIndex = self.environmentPoints(dynamicPointIndexes(j)).get('referencePrimitiveIndex');
                        primitiveTrajectory = self.environmentPrimitives(primitiveIndex).get('trajectory');
                        positions(:,j) = pointTrajectory.RelativeToAbsolutePoint(primitiveTrajectory,t(i),'R3Position');
                    else
                        positions(:,j) = self.environmentPoints(dynamicPointIndexes(j)).get('R3Position',t(i));
                    end
                end
                %plot positions
                h1 = plot3(positions(1,:),positions(2,:),positions(3,:),'k.');
                %plot primitives
%                 for j = 1:staticPrimitiveIndexes
%                     h2{j} = self.environmentPrimitives(staticPrimitiveIndexes(i)).plot()
%                 end

                %draw current timestep, delete
                drawnow
                pause(0)
                if i < nSteps
                    delete(h1)
%                     delete(h2)
                end
            end
        end
    end
end

