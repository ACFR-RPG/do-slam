%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

classdef GeometricObject < SensorObject
    %GEOMETRICOBJECT class instances are used by Sensor class instances to 
    %create a representation of geometric objects in the environment.
    %   Geometric objects have trajectory and parameters (ie cube has pose 
    %   side length
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        trajectory
        parameters
    end
    
    %% 2. Methods
    
end

