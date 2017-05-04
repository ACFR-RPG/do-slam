classdef SensorEnvironment
    %SENSORENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = SensorEnvironment(environment)
            %create points from environmentPoints
            for i = 1:environment.nEnvironmentPoints
            end
            
            %create objects from environmentPrimitives
            for i = 1:environment.nEnvironmentPrimitives
                switch class(environment.get('environmentPrimitives',i))
                    case  'EP_Rectangle'
                        display(environment.get('environmentPrimitives',i).get('environmentPointIndexes'))
                    otherwise
                        error('Error: %s conversion not implemented',class(environment.get('environmentPrimitives',i)))
                end
            end
        end
    end
    
end

