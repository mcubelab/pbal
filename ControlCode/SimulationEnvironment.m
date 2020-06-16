%This class defines a simulation environment which contains variours
%polygonal rigid bodies that make fricitional contacts with one another
%The key purpose of this class is to update the system state by a
%single timestep, then call the necessary update functions for the
%visualizations of the system. As such, it will be necessary for the
%simulation environment to keep track of all the kinematic constraints
%of contact and sliding, as well as the force constraints taht deal with
%contact and friction.
classdef SimulationEnvironment < handle
    
    properties
        rigid_body_list; %list of the rigid body objects in the environment
        constraint_list; %list of the kinematic constraint objects in the system
        
        num_bodies; %the number of rigid bodies
        num_coordinates; %the number of generalized coordinates in the system
        
        num_constraints; %number of kinematic constraints in the environment
        num_lagrange_multipliers; %number of lagrange multipliers in the environment
        %since sticking contact corresponds to two lagrange multipliers
        %(an x and a y constraint), there may be a different number of
        %lagrange multipliers than there are constraints.
        
        
        num_generalized_forces; %number of generalized forces in the system
        generalized_force_list; %list of generalized force objects
        
        dt; %global euler time step
    end
    
    methods
        %Initializes a SimulationEnvironment
        %Makes a bunch of empty lists, sets all lengths to 0
        function obj=SimulationEnvironment()
            %rigid_body_list is a cell array where each element is an
            %instance of PolygonRigidBody
            obj.rigid_body_list={};
            
            %constriant_list is a cell array where each element is an
            %instance of PolygronConstraint, which represents either a
            %sticking or sliding contact constraint, for the time being
            obj.constraint_list={};
            
            obj.generalized_force_list={};
            
            obj.num_bodies=0;
            obj.num_coordinates=0;
            obj.num_constraints=0;
            obj.num_lagrange_multipliers=0;
            obj.num_generalized_forces=0;
            
        end
        
        %A set function for the timestep
        function obj=setdt(obj,dt)
            obj.dt=dt;
        end
        
        %appends a rigid body object to the environment and increments the
        %number of rigid bodies and generalized coordinates
        %at the same time, this assigns a label to the rigid body, and the
        %generalized coordinates of the rigid body
        function addRigidBody(obj,inputBody)
            %incrementing number of bodies
            obj.num_bodies=obj.num_bodies+1;
            
            %assigns label to rigid body
            inputBody.rigid_body_index=obj.num_bodies;
            
            %assigns labels to gen coords of that rigid body
            inputBody.coord_index=3*(obj.num_bodies-1)+(1:3);
            
            %incrementing number of coordinates
            obj.num_coordinates=obj.num_coordinates+3;
            
            %append rigid body to the list
            obj.rigid_body_list{obj.num_bodies}=inputBody;
        end
        
        %appends a kinematic constraint to the environment and increments
        %the number of constraints and lagrange multipliers
        %at the same time, this assigns a label to the kinematic constraint
        %as well as the lagrange multipliers
        function addConstraint(obj,inputConstraint)
            %increment number of constraints
            obj.num_constraints=obj.num_constraints+1;
            
            %assign label to the constraints
            inputConstraint.constraint_index=obj.num_constraints;
            
            %append constraint to the list
            obj.constraint_list{obj.num_constraints}=inputConstraint;
            
            %assign labels to the lagrange multipliers
            inputConstraint.lagrange_multiplier_index=(1:inputConstraint.num_lagrange_multipliers)+obj.num_lagrange_multipliers;
            
            %increment number of lagrange multipliers
            obj.num_lagrange_multipliers=obj.num_lagrange_multipliers+inputConstraint.num_lagrange_multipliers;
        end
        
        %appends a generalized force to the environment
        %and increments the number of generalized forces
        function addGeneralizedForce(obj,inputGeneralizedForce)
            %increment number of gen forces
            obj.num_generalized_forces=obj.num_generalized_forces+1;
            
            %appends a gen force to the environment
            obj.generalized_force_list{obj.num_generalized_forces}=inputGeneralizedForce;
        end
        
        %This computes the accelerations of the generalized coordinates
        %given the state of the system (positions and velocities of all the
        %generalized coordinates), the kinematic constraints, and the
        %generalized forces acting on the system
        %once these accelerations have been computed, they are stored in
        %the respective rigid bodies
        function computeAccelerations(obj)
            
            %BigMatrixM is the mass matrix
            BigMatrixM=zeros(obj.num_coordinates,...
                             obj.num_coordinates);
            
            %BigVectorV are the coriolis and centripetal terms in the Euler-Lagrange equation             
            BigVectorV=zeros(obj.num_coordinates,1);
            
            %BigVectorF is the vector of generalized forces
            BigVectorF=zeros(obj.num_coordinates,1);
            
            %BigMatrixA is the constraint matrix on the accelerations
            %associated with the kinematic constraints
            BigMatrixA=zeros(obj.num_lagrange_multipliers,obj.num_coordinates);
            
            %BigVectorB is the vector of quadratic terms (with respect to
            %the velocities) that shows up in the second derivative of the
            %kinematic constraint equations
            BigVectorB=zeros(obj.num_lagrange_multipliers,1);
            
            %BigMatrixBlank a matrix of zeros with the correct dimensions
            %to make everything else work out
            BigMatrixBlank=zeros(obj.num_lagrange_multipliers,obj.num_lagrange_multipliers);
            
            %Iterate through each rigid body and build the associated mass
            %matrix which is just a block diagonal matrix where each block
            %is the mass matrix for the individual rigid body
            %Also build up the vector of centripetal/coriolis terms which
            %is just all the individual centripetal/coriolis vectors
            %appended to on another
            for n=1:obj.num_bodies
                [M,V] = obj.rigid_body_list{n}.LagrangeVM();
                BigMatrixM(obj.rigid_body_list{n}.coord_index,obj.rigid_body_list{n}.coord_index)=M;
                BigVectorV(obj.rigid_body_list{n}.coord_index)=V;
            end
            
            %Summs together each generalized force vector to fine the total
            %generalized force acting on the system (not taking into
            %account constraint forces)
            for n=1:obj.num_generalized_forces
                BigVectorF=BigVectorF+obj.generalized_force_list{n}.generateForce(obj.num_coordinates);
            end
            
            %Iterate through each kinematic constraint, and build the
            %matrix associated with the constraints on the accelerations,
            %to build this, we generate each row (or block of rows) by
            %calling the associated block function for the constraint
            %we do the same exact thing for the vector of quadratic terms
            %with respect to the velocities in the second derivative of the
            %constraint equation
            for n=1:obj.num_constraints
                CurrentConstraint=obj.constraint_list{n};
                for m=1:CurrentConstraint.num_lagrange_multipliers
                    [A,B]=CurrentConstraint.generateBlock(obj.num_lagrange_multipliers);
                    BigMatrixA(obj.num_coordinates+CurrentConstraint.lagrange_multiplier_index,:)=A;
                    BigVectorB(obj.num_coordinates+CurrentConstraint.lagrange_multiplier_index)=B;
                end
            end
            
            
            %We build a linear equation (linear with respect to the
            %accelerations of the generalized coordinates and the lagrange
            %multipliers) of the form:
            %BigMatrix*[accel,lambda]^T=BigVector
            BigMatrix=[[BigMatrixM,-BigMatrixA'];[BigMatrixA,BigMatrixBlank]];
            BigVector=[BigVectorV+BigVectorF;BigVectorB];
            
            %Solve for the accelerations
            AccelVector=BigMatrix\BigVector;
            AccelVector=AccelVector(1:obj.num_coordinates);
            
            %assign the solved accelerations to the associated
            %accelerations of the rigid bodies
            for n=1:obj.num_bodies
                AccelVals=AccelVector(obj.rigid_body_list{n}.coord_index);
                obj.rigid_body_list{n}.set_a_and_alpha(AccelVals(1:2),AccelVals(3));
            end
        end
        
        %This function does a simple forward-euler update of the 
        %positions and velocities of the rigid bodies in the simulation
        %using the time step parameter dt
        function EulerUpdate(obj)
            %iterate through each rigid body, and perform an euler step
            for n=1:obj.num_bodies
                obj.rigid_body_list{n}.EulerUpdate(obj.dt);
            end
        end
        
        function initialize_visualization(obj)
            
        end
        
        %This function updates the plot for each of the rigid bodies
        %and kinematic constraints
        function update_visualization(obj)
            %iterate through each rigid body, and update the visualization
            for n=1:obj.num_bodies
                obj.rigid_body_list{n}.update_visualization();
            end
            
            %iterate through each constraint, and update the visualization
            for n=1:obj.num_constraints
                obj.constraint_list{n}.update_visualization();
            end
        end
        
        %This function projects the current positions and velocities of
        %the rigid bodies onto the constraint manifold defined by the
        %kinematic constraints of the system
        function ConstraintProjection(obj)
            
        end
    end
    
end