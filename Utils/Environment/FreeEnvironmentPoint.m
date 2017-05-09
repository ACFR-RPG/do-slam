classdef FreeEnvironmentPoint < EnvironmentPoint
    %EP_POINT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'private', SetAccess = 'private')
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = FreeEnvironmentPoint(trajectory,index)
            switch nargin
                case 0
                case 2
                    self.trajectory = trajectory;
                    self.index      = index;
            end
        end
        
    end
    
    % Getter & Setter
    methods(Access = public) %set to protected later??
        function out = getSwitch(self,property,varargin)
            switch property
                case 'R3Position'
                    out = self.trajectory.get('R3Position',varargin{1});
                otherwise
                    out = self.(property);
            end
        	
        end
        
        function self = setSwitch(self,property,value)
        	self.(property) = value;
        end
    end
    
    
    
end

