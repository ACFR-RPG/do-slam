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
            switch property
                case 'environmentPoints'
                    if numel(varargin)==1
                        out = self.environmentPoints(varargin{1});
                    else
                        out = self.environmentPoints;
                    end
                case 'environmentPrimitives'
                    if numel(varargin)==1
                        out = self.environmentPrimitives(varargin{1});
                    else
                        out = self.environmentPrimitives;
                    end
                otherwise
                    out = self.(property);
            end
        	
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
            
            %initialise points
            points(nPoints) = EnvironmentPoint();
            for i = 1:nPoints
                iRelativePoint = GP_Point(positionsRelative(:,i),parameterisation);
                points(i).set('trajectory',RelativePointTrajectory(trajectory,iRelativePoint));
            end
            
            %construct primitive
            primitive = EP_Default();
            primitive.set('trajectory',trajectory);
            
            %pair
            self.addPrimitiveAndPoints(primitive,points);
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
                        
            %initialise points
            rectanglePoints(nPoints) = EnvironmentPoint();
            for i = 1:nPoints
                iRelativePoint = GP_Point(rectanglePositionsRelative(:,i));
                rectanglePoints(i).set('trajectory',RelativePointTrajectory(rectangleTrajectory,iRelativePoint));
            end
            
            %initialise EP_Rectangle primitive
            rectanglePrimitive = EP_Rectangle(sideLengths,rectangleTrajectory);
            
            %pair
            self.addPrimitiveAndPoints(rectanglePrimitive,rectanglePoints);
            
        end
       
    end
    
    % Add primitive & points
    methods(Access = private, Hidden = true)
        function self = addPrimitiveAndPoints(self,primitive,points)
            primitiveIndex = self.nEnvironmentPrimitives + 1;
            pointIndexes   = self.nEnvironmentPoints + 1:self.nEnvironmentPoints + numel(points);
            
            %set primitive indexes
            primitive.set('index',primitiveIndex);
            primitive.set('pointIndexes',pointIndexes);
            
            %set point indexes
            points.set('index',pointIndexes);
            points.set('primitiveIndexes',repmat(primitiveIndex,1,numel(points)));
            
            %add to self
            self.environmentPrimitives = [self.environmentPrimitives primitive];
            self.environmentPoints = [self.environmentPoints points];
        end
    end

    % plot
    methods(Access = public)
        function plot(self,t)
            %identify dynamic points and dynamic primitives
            staticPointLogical      = self.get('environmentPoints').get('static');
            staticPrimitiveLogical  = self.get('environmentPrimitives').get('static');
            dynamicPointLogical     = ~staticPointLogical;
            dynamicPrimitiveLogical = ~staticPrimitiveLogical;
            staticPointIndexes      = find(staticPointLogical);
            staticPrimitiveIndexes  = find(staticPrimitiveLogical);
            dynamicPointIndexes     = find(dynamicPointLogical);
            dynamicPrimitiveIndexes = find(dynamicPrimitiveLogical);
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
                positions(:,i) = self.environmentPoints(staticPointIndexes(i)).get('R3Position',0);
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
                    positions(:,j) = self.environmentPoints(dynamicPointIndexes(j)).get('R3Position',t(i));
                end
                %plot positions
                h1 = plot3(positions(1,:),positions(2,:),positions(3,:),'k.');
                %plot primitives
%                 for j = 1:dynamicPrimitiveIndexes
%                     h2{j} = self.environmentPrimitives(dynamicPrimitiveIndexes(i)).plot()
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
