%This class defines a simulation environment which contains variours
%polygonal rigid bodies that make fricitional contacts with one another
%The key purpose of this class is to update the system state by a
%single timestep, then call the necessary update functions for the
%visualizations of the system. As such, it will be necessary for the
%simulation environment to keep track of all the kinematic constraints
%of contact and sliding, as well as the force constraints taht deal with
%contact and friction.
classdef SimulationEnvironment < handle
    
    
end