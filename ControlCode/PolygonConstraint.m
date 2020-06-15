
classdef PolygonConstraint < handle
    
    properties
        rigidBody1;
        rigidBody2;
        
        pin1;
        pin2;
        
        pout;
    
        constraint_index;
        num_lagrange_multipliers;
        
        lagrange_multiplier_index;
        
        ConstraintType;
    end
    
    methods
        function obj=PolygonConstraint()
            obj.ConstraintType=0;
            obj.num_lagrange_multipliers=0;
        end
        
        function StickingContactOneBody(obj,Body1,pin1,pout)
            obj.rigidBody1=Body1;
            obj.pin1=pin1;
            obj.pout=pout;
            
            obj.ConstraintType=1;
            
            obj.num_lagrange_multipliers=2;
        end
        
        function StickingContactTwoBodies(obj,Body1,pin1,Body2,pin2)
            obj.rigidBody1=Body1;
            obj.pin1=pin1;
            
            obj.rigidBody2=Body2;
            obj.pin2=pin2;
            
            obj.ConstraintType=2;
            
            obj.num_lagrange_multipliers=2;
        end
        
        
        function [A,B] = generateBlock(obj,numLagrange)
            if obj.ConstraintType==0
                disp('Error: Polygon Constraint Never Initialized');
            end
            
            if obj.ConstraintType==1
                [A,B] = obj.generateBlockStickingContactOneBody(numLagrange);
            end
            
            if obj.ConstraintType==2
                [A,B] = obj.generateBlockStickingContactTwoBodies(numLagrange);
            end
        end
        
        function [A,B] = generateBlockStickingContactOneBody(obj,numLagrange)
            [x1,y1,Dx1,Dy1,Hx1,Hy1]=obj.rigidBody1.rigid_body_position_derivatives(obj.pin1);
           
            A=zeros(2,numLagrange);
            A(1,obj.rigidBody1.coord_index)=Dx1;
            A(2,obj.rigidBody1.coord_index)=Dy1;
            
            [v1,omega1]=obj.rigidBody1.get_v_and_omega();
            V1=[v1;omega1];
            
            B(1)=-V1'*Hx1*V1;
            B(2)=-V1'*Hy1*V1;
        end
        
        function [A,B] = generateBlockStickingContactTwoBodies(obj,numLagrange)
            [x1,y1,Dx1,Dy1,Hx1,Hy1]=obj.rigidBody1.rigid_body_position_derivatives(obj.pin1);
            [x2,y2,Dx2,Dy2,Hx2,Hy2]=obj.rigidBody2.rigid_body_position_derivatives(obj.pin2);
            
            A=zeros(2,numLagrange);
            A(1,obj.rigidBody1.coord_index)=Dx1;
            A(2,obj.rigidBody1.coord_index)=Dy1;
            A(1,obj.rigidBody2.coord_index)=-Dx2;
            A(2,obj.rigidBody2.coord_index)=-Dy2;
            
            [v1,omega1]=obj.rigidBody1.get_v_and_omega();
            V1=[v1;omega1];
            
            [v2,omega2]=obj.rigidBody2.get_v_and_omega();
            V2=[v2;omega2];
            
            B(1)=-V1'*Hx1*V1+V2'*Hx2*V2;
            B(2)=-V1'*Hy1*V1+V2'*Hy2*V2;
        end
        
    end
    
    
    
end