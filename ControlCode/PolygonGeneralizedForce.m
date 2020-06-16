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
        
        GeneralizedForceType; %the type of generalized force we care about
        %0 corresponds to nothing
        %1 corresponds to the gravitational force
        %2
    end
    
    methods
        %Intializes an empty generalized force, sets type to 0 (blank)
        function obj=PolygonGeneralizedForce()
            obj.GeneralizedForceType=0;
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
        
        %Stores relevant inforation for gravity, and sets type to 1
        function gravity(obj,rigidBody,g)
            obj.rigidBody1=rigidBody;
            obj.g=g;
            obj.GeneralizedForceType=1;
        end
    end
end