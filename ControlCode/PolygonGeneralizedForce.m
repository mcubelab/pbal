%Describes the properties of a generalized force within a multi-rigidbody
%systme
classdef PolygonGeneralizedForce < handle
    properties
        rigidBody1; %First rigid body object that the generalized force affects
        rigidBody2; %Second rigid body object that the generalized force affects (may be optional)
        
        pin1; %Relevant material point on the first rigid body (in the body frame)
        pin2; %Relevant material point on the second rigid body (in the body frame)
        
        pout; %Point that we may care about in the world frame
        
        g; %gravity acceleration vector
        external_wrench_val; %external wrenchn exerted on the object 
        %wrench has form [fx,fy,tau]
        
        is_conservative; %1 if conservative, 0 if not conservative
        
        GeneralizedForceType; %the type of generalized force we care about
        %0 corresponds to nothing
        %1 corresponds to the gravitational force
        %2 corresponds to an external wrench
    end
    
    methods
        %Intializes an empty generalized force, sets type to 0 (blank)
        function obj=PolygonGeneralizedForce()
            obj.GeneralizedForceType=0;
            obj.is_conservative=0;
        end
        
        %Generate the generalized force vector associated with each
        %generalized coordinate
        %num_coordinates is the number of generalized coordinates in the
        %simulation environment, this is needed to know the length of the
        %associated generalized force vector
        function F = generateForce(obj,num_coordinates)
            %call the associated generateForce function for each type of
            %generalized force
            if obj.GeneralizedForceType==0
                disp('Error: Generalized Force Never Initialized');
            end
            
            if obj.GeneralizedForceType==1
                F = obj.generateForceGravity(num_coordinates);
            end
            
            if obj.GeneralizedForceType==2
               F = obj.generateForceExternalWrench(num_coordinates); 
            end
        end
        
        function U = computePotential(obj)
            %call the associated generateForce function for each type of
            %generalized force
            if obj.GeneralizedForceType==0
                disp('Error: Generalized Force Never Initialized');
            end
            
            if obj.is_conservative==0
                disp('Error: Force is not conservative');
            end
            
            if obj.GeneralizedForceType==1
                U = obj.computePotentialGravity();
            end
        end
        
        %Generate the generalized force vector associated with each
        %generalized coordinate
        %num_coordinates is the number of generalized coordinates in the
        %simulation environment, this is needed to know the length of the
        %associated generalized force vector
        
        %Specifically for the gravitational force
        function F = generateForceGravity(obj,num_coordinates)
            F=zeros(num_coordinates,1);
            [~,~,Dx,Dy,~,~]=obj.rigidBody1.rigid_body_CM_derivatives();
            
            F(obj.rigidBody1.coord_index)=obj.rigidBody1.Mass*(obj.g(1)*Dx'+obj.g(2)*Dy');
            
        end
        
        %Computes the potential energy associated with the gravitational
        %force
        function U = computePotentialGravity(obj)
            [x,y,~,~,~,~]=obj.rigidBody1.rigid_body_CM_derivatives();
            U=-obj.rigidBody1.Mass*(obj.g(1)*x+obj.g(2)*y);
        end
        
        
        %Stores relevant information for gravity, and sets type to 1
        function gravity(obj,rigidBody,g)
            obj.rigidBody1=rigidBody;
            obj.g=g;
            obj.GeneralizedForceType=1;
            obj.is_conservative=1;
        end
        
        %Sets the generalized force to correspond to some external wrench
        %exerted by the object at some material point on the rigid body
        function external_wrench(obj,rigidBody1,pin1)
           obj.rigidBody1=rigidBody1;
           obj.pin1=pin1;
           obj.GeneralizedForceType=2;
           obj.is_conservative=0;
        end
        
        %Sets the current value of the external wrench to wrench_in
        function set_wrench_value(obj,wrench_in)
            obj.external_wrench_val=wrench_in;
        end
        
        %Generate the generalized force vector associated with each
        %generalized coordinate
        %num_coordinates is the number of generalized coordinates in the
        %simulation environment, this is needed to know the length of the
        %associated generalized force vector
        
        %Specifically for the an external wrench exerted at a material
        %point of a rigid body of the system.
        function F = generateForceExternalWrench(obj,num_coordinates)
            F=zeros(num_coordinates,1);

            [~,~,Dx,Dy,~,~]=obj.rigidBody1.rigid_body_position_derivatives(obj.pin1);
            F(obj.rigidBody1.coord_index)=obj.external_wrench_val(1)*Dx'+obj.external_wrench_val(2)*Dy'+obj.external_wrench_val(3)*[0;0;1];
        end
    end
end