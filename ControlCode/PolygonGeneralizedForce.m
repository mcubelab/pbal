classdef PolygonGeneralizedForce < handle
    properties
        rigidBody1;
        rigidBody2;
        
        pin1;
        pin2;
        
        pout;
        
        g;
        
        GeneralizedForceType;
    end
    
    methods
        function obj=PolygonGeneralizedForce()
            obj.GeneralizedForceType=0;
        end
        
        function F = generateForce(obj,num_coordinates)
            if obj.GeneralizedForceType==0
                disp('Error: Generalized Force Never Initialized');
            end
            
            if obj.GeneralizedForceType==1
                F = obj.generateForceGravity(num_coordinates);
            end
        end
        
        function F = generateForceGravity(obj,num_coordinates)
            F=zeros(num_coordinates,1);
            [~,~,Dx,Dy,~,~]=obj.rigidBody1.rigid_body_CM_derivatives();
            
            F(obj.rigidBody1.coord_index)=obj.rigidBody1.Mass*(obj.g(1)*Dx'+obj.g(2)*Dy');
        end
        
        function gravity(obj,rigidBody,g)
            obj.rigidBody1=rigidBody;
            obj.g=g;
        end
    end
end