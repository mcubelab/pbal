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
        rigid_body_list;
        constraint_list;
        
        num_bodies;
        num_coordinates;
        
        num_constraints;
        num_lagrange_multipliers;
        
        num_generalized_forces;
        generalized_force_list;
        
        dt;
    end
    
    methods
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
        
        function obj=setdt(obj,dt)
            obj.dt=dt;
        end
        
        function addRigidBody(obj,inputBody)
            obj.num_bodies=obj.num_bodies+1;
            inputBody.rigid_body_index=obj.num_bodies;
            inputBody.coord_index=3*(obj.num_bodies-1)+(1:3);
            
            obj.num_coordinates=obj.num_coordinates+3;
            obj.rigid_body_list{obj.num_bodies}=inputBody;
        end
        
        function addConstraint(obj,inputConstraint)
            obj.num_constraints=obj.num_constraints+1;
            
            inputConstraint.constraint_index=obj.num_constraints;
            obj.constraint_list{obj.num_constraints}=inputConstraint;
            
            inputConstraint.lagrange_multiplier_index=(1:inputConstraint.num_lagrange_multipliers)+obj.num_lagrange_multipliers;
            
            obj.num_lagrange_multipliers=obj.num_lagrange_multipliers+inputConstraint.num_lagrange_multipliers;
        end
        
        function addGeneralizedForce(obj,inputGeneralizedForce)
            obj.num_generalized_forces=obj.num_generalized_forces+1;
            obj.generalized_force_list{obj.num_generalized_forces}=inputGeneralizedForce;
        end
        
        function computeAccelerations(obj)
            BigMatrixM=zeros(obj.num_coordinates,...
                             obj.num_coordinates);
                         
            BigVectorV=zeros(obj.num_coordinates,1);
            
            BigVectorF=zeros(obj.num_coordinates,1);
            
            BigMatrixA=zeros(obj.num_lagrange_multipliers,obj.num_coordinates);
            BigVectorB=zeros(obj.num_lagrange_multipliers,1);
            
            BigMatrixBlank=zeros(obj.num_lagrange_multipliers,obj.num_lagrange_multipliers);
            for n=1:obj.num_bodies
                [M,V] = obj.rigid_body_list{n}.LagrangeVM();
                BigMatrixM(obj.rigid_body_list{n}.coord_index,obj.rigid_body_list{n}.coord_index)=M;
                BigVectorV(obj.rigid_body_list{n}.coord_index)=V;
            end
            
            for n=1:obj.num_generalized_forces
                BigVectorF=BigVectorF+obj.generalized_force_list{n}.generateForce(obj.num_coordinates);
            end
            
            for n=1:obj.num_constraints
                CurrentConstraint=obj.constraint_list{n};
                for m=1:CurrentConstraint.num_lagrange_multipliers
                    [A,B]=CurrentConstraint.generateBlock(obj.num_lagrange_multipliers);
                    BigMatrixA(obj.num_coordinates+CurrentConstraint.lagrange_multiplier_index,:)=A;
                    BigVectorB(obj.num_coordinates+CurrentConstraint.lagrange_multiplier_index)=B;
                end
            end
            
            BigMatrix=[[BigMatrixM,-BigMatrixA'];[BigMatrixA,BigMatrixBlank]];
            BigVector=[BigVectorV+BigVectorF;BigVectorB];
            
            AccelVector=BigMatrix\BigVector;
            AccelVector=AccelVector(1:obj.num_coordinates);
            
            for n=1:obj.num_bodies
                AccelVals=AccelVector(obj.rigid_body_list{n}.coord_index);
                obj.rigid_body_list{n}.set_a_and_alpha(AccelVals(1:2),AccelVals(3));
            end
        end
        
        %This function does a simple forward-euler update of the 
        %positions and velocities of the rigid bodies in the simulation
        %using the time step parameter dt
        function EulerUpdate(obj)
            for n=1:obj.num_bodies
                obj.rigid_body_list{n}.EulerUpdate(obj.dt);
            end
        end
        
        function update_visualization(obj)
            for n=1:obj.num_bodies
                obj.rigid_body_list{n}.update_visualization();
            end
            
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