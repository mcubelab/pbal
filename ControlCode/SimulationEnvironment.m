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
        
        
        q0;     %initial value of the generalized coordinates
        q_dot0; %inital value of the derivative of the genearlized coords
        U0;     %initial value of potential energy
        T0;     %initial value of kinetic energy
        
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
        
        function acceleration_vector = computeAccelerations_withInput(obj,q,v)
            obj.assign_coordinate_vector(q);
            obj.assign_velocity_vector(v);
            
            obj.computeAccelerations();
            
            acceleration_vector=obj.build_acceleration_vector();
        end
        
        function BraunUpdate(obj)
            %BigMatrixM is the mass matrix
            BigMatrixM=zeros(obj.num_coordinates,...
                             obj.num_coordinates);
            
            %BigVectorV are the coriolis and centripetal terms in the Euler-Lagrange equation             
            BigVectorV=zeros(obj.num_coordinates,1);
            
            %BigVectorV are the coriolis and centripetal terms in the Euler-Lagrange equation             
            BigVectorV1=zeros(obj.num_coordinates,1);
            
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
                [M,V,V1,~] = obj.rigid_body_list{n}.LagrangeVM();
                BigMatrixM(obj.rigid_body_list{n}.coord_index,obj.rigid_body_list{n}.coord_index)=M;
                BigVectorV(obj.rigid_body_list{n}.coord_index)=V;
                BigVectorV1(obj.rigid_body_list{n}.coord_index)=V1;
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
                    [A,B,~]=CurrentConstraint.generateBlock(obj.num_coordinates);
                    BigMatrixA(CurrentConstraint.lagrange_multiplier_index,:)=A;
                    BigVectorB(CurrentConstraint.lagrange_multiplier_index)=B;
                end
            end
            
            q=obj.build_coordinate_vector();
            q_dot=obj.build_velocity_vector();
            v=q_dot;
            
            M=BigMatrixM;
            Q=BigVectorF+BigVectorV;
            R=chol(M);
            Aq=BigMatrixA;
            bq=0*BigVectorB;
            Av=Aq;
            bv=BigVectorB;
            Phiq=obj.EvalConstraintError();
            dPhiqdt=Aq*q_dot;
            dt=obj.dt;
            Ae=q_dot'*M;
            be=-q_dot'*(BigVectorF+BigVectorV1);
            Phi_e=obj.EnergyChange();
            
            
            Cq=Aq/R;
            Cv=Av/R;
            
            Cq_plus=pinv(Cq);
            Cv_plus=pinv(Cv);
            
            an=M\Q;
            
            Ce=Ae/R;
            Nv=eye(max(size(Cv_plus*Cv)))-Cv_plus*Cv;
            
            bev=be-Ce*Cv_plus*bv;
            Aev=Ae-Ce*Cv_plus*Av;
            Phi_ev=Phi_e;
            
            q_update=q+v*dt+R\Cq_plus*((bq-Aq*v)*dt-Phiq);
            v_update=v+an*dt+R\Cv_plus*((bv-Av*an)*dt-dPhiqdt)...
                +R\Nv*pinv(Ce*Nv)*((bev-Aev*an)*dt-Phi_ev);
            
            obj.assign_coordinate_vector(q_update);
            obj.assign_velocity_vector(v_update);
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
                [M,V,~,~] = obj.rigid_body_list{n}.LagrangeVM();
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
                    [A,B,~]=CurrentConstraint.generateBlock(obj.num_coordinates);
                    BigMatrixA(CurrentConstraint.lagrange_multiplier_index,:)=A;
                    BigVectorB(CurrentConstraint.lagrange_multiplier_index)=B;
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
            obj.assign_acceleration_vector(AccelVector);
        end
        
        % This function builds forward dynamics of the form x_dot = f(x, u)
        % where x_dot = [vel, accel], x = [pos, vel], and u is ...
        function forward_dynamics(obj)

            
        end
        
        %Takes a snapshot of initial position/velocity
        %as well as initial kinetic and potential energy
        function saveInitialState(obj)
            obj.q0    =obj.build_coordinate_vector();
            obj.q_dot0=obj.build_velocity_vector();
                  
            [T,U]=obj.computeEnergies();
            obj.T0=T;
            obj.U0=U;
        end
        
        %Computes the current kinetic and potential energy of the system.
        function [T,U]=computeEnergies(obj)
            %BigMatrixM is the mass matrix
            BigMatrixM=zeros(obj.num_coordinates,...
                             obj.num_coordinates);
                         
            %Iterate through each rigid body and build the associated mass
            %matrix which is just a block diagonal matrix where each block
            %is the mass matrix for the individual rigid body
            %Also build up the vector of centripetal/coriolis terms which
            %is just all the individual centripetal/coriolis vectors
            %appended to on another
            for n=1:obj.num_bodies
                [M,~] = obj.rigid_body_list{n}.LagrangeVM();
                BigMatrixM(obj.rigid_body_list{n}.coord_index,obj.rigid_body_list{n}.coord_index)=M;
            end
                
            T=.5*(obj.q_dot0)'*(BigMatrixM*obj.q_dot0);
            
            U=0;
            
            %Summs together the potential energy for each of the
            %generalized forces
            for n=1:obj.num_generalized_forces
                if obj.generalized_force_list{n}.is_conservative==1
                    U=U+obj.generalized_force_list{n}.computePotential();
                end
            end  
        end
        
        %Computes the change in mechanical energy from the initial state
        function DeltaE=EnergyChange(obj)
            [T,U]=obj.computeEnergies()
            DeltaE=T+U-obj.T0-obj.U0;
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
        
        %This function initializes the plots of each of the rigid bodies
        %and kinematic constraints
        function initialize_visualization(obj)
            %iterate through each rigid body, and initialize the visualization
            for n=1:obj.num_bodies
                obj.rigid_body_list{n}.initialize_visualization();
            end
            
            %iterate through each constraint, and initialize the visualization
            for n=1:obj.num_constraints
                obj.constraint_list{n}.initialize_visualization();
            end
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
        
        %Builds the vector of generalized coordinates across all rigid
        %bodies
        function gen_coord_vector=build_coordinate_vector(obj)
            gen_coord_vector=zeros(obj.num_coordinates,1);
            for n=1:obj.num_bodies
                [p,theta]=obj.rigid_body_list{n}.get_p_and_theta();
                gen_coord_vector(obj.rigid_body_list{n}.coord_index)=[p;theta];
            end
        end
        
        %Builds the vector of velocities of the generalized coordinates 
        %across all rigid bodies
        function velocity_vector=build_velocity_vector(obj)
            velocity_vector=zeros(obj.num_coordinates,1);
            for n=1:obj.num_bodies
                [v,omega]=obj.rigid_body_list{n}.get_v_and_omega();
                velocity_vector(obj.rigid_body_list{n}.coord_index)=[v;omega];
            end
        end
        
        %Builds the vector of accelerations of the generalized coordinates 
        %across all rigid bodies
        function acceleration_vector=build_acceleration_vector(obj)
            acceleration_vector=zeros(obj.num_coordinates,1);
            for n=1:obj.num_bodies
                [a,alpha]=obj.rigid_body_list{n}.set_a_and_alpha();
                acceleration_vector(obj.rigid_body_list{n}.coord_index)=[a;alpha];
            end
        end
        
        %updates the values of generalized coordinates across all rigid
        %bodies
        function assign_coordinate_vector(obj,gen_coord_vector)
            for n=1:obj.num_bodies
                gen_coord_vector_body=gen_coord_vector(obj.rigid_body_list{n}.coord_index);
                obj.rigid_body_list{n}.set_p_and_theta(gen_coord_vector_body(1:2),gen_coord_vector_body(3));
            end
        end
        
        %updates the values of generalized velocities across all rigid
        %bodies
        function assign_velocity_vector(obj,velocity_vector)
            for n=1:obj.num_bodies
                velocity_vector_body=velocity_vector(obj.rigid_body_list{n}.coord_index);
                obj.rigid_body_list{n}.set_v_and_omega(velocity_vector_body(1:2),velocity_vector_body(3));
            end
        end
        
        %updates the values of generalized accelerations across all rigid
        %bodies
        function assign_acceleration_vector(obj,acceleration_vector)
            for n=1:obj.num_bodies
                acceleration_vector_body=acceleration_vector(obj.rigid_body_list{n}.coord_index);
                obj.rigid_body_list{n}.set_a_and_alpha(acceleration_vector_body(1:2),acceleration_vector_body(3));
            end
        end
             
        function ConstraintErrorOut=EvalConstraintError(obj)
            
            %BigVectorB is the vector of constraint errors
            BigVectorB=zeros(obj.num_lagrange_multipliers,1);
                
            for n=1:obj.num_constraints
                CurrentConstraint=obj.constraint_list{n};
                for m=1:CurrentConstraint.num_lagrange_multipliers
                    [A,~,ConstraintError]=CurrentConstraint.generateBlock(obj.num_coordinates);
                    BigVectorB(CurrentConstraint.lagrange_multiplier_index)=ConstraintError;
                end
            end
            
            ConstraintErrorOut=BigVectorB;
        end
        
        %This function projects the current positions and velocities of
        %the rigid bodies onto the constraint manifold defined by the
        %kinematic constraints of the system
        function ConstraintProjection(obj)
            
            %Perform 5 newton steps
            for count=1:5
                %BigMatrixA is the constraint matrix on the velocities
                %(jacobian of constraint function)
                %associated with the kinematic constraints
                BigMatrixA=zeros(obj.num_lagrange_multipliers,obj.num_coordinates);

                %BigVectorB is the vector of constraint errors
                BigVectorB=zeros(obj.num_lagrange_multipliers,1);


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
                        
                        [A,~,ConstraintError]=CurrentConstraint.generateBlock(obj.num_coordinates);
                        BigMatrixA(CurrentConstraint.lagrange_multiplier_index,:)=A;
                        BigVectorB(CurrentConstraint.lagrange_multiplier_index)=ConstraintError;
                    end
                end

                %Compute the newton step, then
                %assign the solved gen coord changes to the associated
                %generalized coordinates of the rigid bodies
                obj.assign_coordinate_vector(obj.build_coordinate_vector()-BigMatrixA\BigVectorB);

            end
            
            
            %BigMatrixA is the constraint matrix on the velocities
            %(jacobian of constraint function)
            %associated with the kinematic constraints
            BigMatrixA=zeros(obj.num_lagrange_multipliers,obj.num_coordinates);
            
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
                    [A,~,~]=CurrentConstraint.generateBlock(obj.num_coordinates);
                    BigMatrixA(CurrentConstraint.lagrange_multiplier_index,:)=A;  
                end
            end
            

            GenVelocities=obj.build_velocity_vector();
            
            BigMatrix=[[eye(obj.num_coordinates,obj.num_coordinates),BigMatrixA'];[BigMatrixA,zeros(obj.num_lagrange_multipliers)]];
            BigVector=[zeros(obj.num_coordinates,1);-BigMatrixA*GenVelocities];
            
            dGenVelocities=BigMatrix\BigVector;
            dGenVelocities=dGenVelocities(1:obj.num_coordinates);
            
            obj.assign_velocity_vector(GenVelocities+dGenVelocities);
        end
        
        %Computes the accelerations of the system, does an Euler step
        %then projects the system onto the constraint manifold
        function update_project(obj)
            obj.computeAccelerations();
            obj.EulerUpdate();
            obj.ConstraintProjection();
        end
        
        %Computes the accelerations of the system, does an Euler step
        %then projects the system onto the constraint manifold
        %finally, updates the visualization of the system
        function update_project_visualize(obj)
            obj.update_project();
            obj.update_visualization();
        end
        
        %then projects the system onto the constraint manifold
        %then updates the visualization of the system
        function project_visualize(obj)
            obj.ConstraintProjection();
            obj.update_visualization();
        end
    end
    
end