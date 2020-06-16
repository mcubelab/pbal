
classdef PolygonConstraint < handle
    
    properties
        rigidBody1; %first rigid body object used by the constraint
        rigidBody2; %second rigid body object used by constraint (may not be used)
        
        pin1; %material point referenced on the first rigid body (may not be used) (in body frame)
        pin2; %material point referenced on the second rigid body (may not be used) (in body frame)
        
        pout; %point used in the world frame
    
        constraint_index; %label assigned to this constraint
        num_lagrange_multipliers; %number of lagrange multipliers associated with this constraint (usually 1 or 2)
        
        lagrange_multiplier_index; %labels assigned to the lagrange multipliers
        %takes the form of just one integer, or a sequence of consecutive
        %integers
        
        ConstraintType; %color/type of constraint, just an integer where 0 corresponds to nothing
        %1 sticking contact with one rigid body, and 1 point in world frame
        %2 sticking contact between two rigid bodies
        %3
        %4
    end
    
    methods
        %initializes blank object
        function obj=PolygonConstraint()
            obj.ConstraintType=0;
            obj.num_lagrange_multipliers=0;
        end
        
        %stores information associated with a sticking contact between a
        %material point on a single rigid body and a point in the world
        %frame (pivot constraint)
        %Body1 is the rigid body object
        %pin1 is the material point on Body1, in the body frame
        %pout is the material point in the world frame
        function StickingContactOneBody(obj,Body1,pin1,pout)
            obj.rigidBody1=Body1;
            obj.pin1=pin1;
            obj.pout=pout;
            
            obj.ConstraintType=1; %assigns the sticking constraint label with 1 rigid body
            
            obj.num_lagrange_multipliers=2; %x and y constraints ->2 multipliers
        end
        
        %stores information associated with a sticking contact between two
        %rigid bodies at 1 material point on each respective rigid body
        %Body1 is the first rigid body object
        %pin1 is the material point on Body1, in the body1 frame
        %Body2 is the second rigid body object
        %pin2 is the material point on Body2, in the body2 frame
        function StickingContactTwoBodies(obj,Body1,pin1,Body2,pin2)
            obj.rigidBody1=Body1;
            obj.pin1=pin1;
            
            obj.rigidBody2=Body2;
            obj.pin2=pin2;
            
            obj.ConstraintType=2; %assigns the sticking constraint label with 2 rigid bodies
            
            obj.num_lagrange_multipliers=2; %x and y constriants -> 2 multipliers
        end
        
        %this builds the linear equation associated with the acceleration
        %form of the kinematic constraints, i.e
        %A(q,dq/dt)*d^2q/dt^2=B(q,dq/dt)
        %where q is the vector of generalized coordinates, B is a vector
        %associated with the quadratic velocity terms that appear when the
        %constraint is differentiated twice with respect to time
        %numLagrange is the total number of lagrange multipliers in the
        %simulation environment, which is needed to determine width of the
        %A matrix!
        function [A,B] = generateBlock(obj,numLagrange)
            
            %call the associated generateBlock function for each type of
            %constraint
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
        
        
         %this builds the linear equation associated with the acceleration
        %form of the kinematic constraints, i.e
        %A(q,dq/dt)*d^2q/dt^2=B(q,dq/dt)
        %where q is the vector of generalized coordinates, B is a vector
        %associated with the quadratic velocity terms that appear when the
        %constraint is differentiated twice with respect to time
        %numLagrange is the total number of lagrange multipliers in the
        %simulation environment, which is needed to determine width of the
        %A matrix!
        
        %this is specifically for sticking contact with one rigid body, and 1 point in world frame
        function [A,B] = generateBlockStickingContactOneBody(obj,numLagrange)
            [x1,y1,Dx1,Dy1,Hx1,Hy1]=obj.rigidBody1.rigid_body_position_derivatives(obj.pin1);
           
            A=zeros(2,numLagrange);
            %the A matrix is built from the gradients of the x and y
            %coordinates of the material point that we care about, where we
            %place those gradients in their correct indices in the A matrix
            %using the assigned global labels for those generalized coordinates
            A(1,obj.rigidBody1.coord_index)=Dx1; 
            A(2,obj.rigidBody1.coord_index)=Dy1;
            
            %given the current state (velocity and position) of the rigid
            %body in question, compute the quadratic terms with respect to
            %velocity for both the x and y coordinates
            [v1,omega1]=obj.rigidBody1.get_v_and_omega();
            V1=[v1;omega1];
            
            B(1)=-V1'*Hx1*V1;
            B(2)=-V1'*Hy1*V1;
        end
        
        %this builds the linear equation associated with the acceleration
        %form of the kinematic constraints, i.e
        %A(q,dq/dt)*d^2q/dt^2=B(q,dq/dt)
        %where q is the vector of generalized coordinates, B is a vector
        %associated with the quadratic velocity terms that appear when the
        %constraint is differentiated twice with respect to time
        %numLagrange is the total number of lagrange multipliers in the
        %simulation environment, which is needed to determine width of the
        %A matrix!
        
        %this is specifically for sticking contact between two rigid bodies
        function [A,B] = generateBlockStickingContactTwoBodies(obj,numLagrange)
            [x1,y1,Dx1,Dy1,Hx1,Hy1]=obj.rigidBody1.rigid_body_position_derivatives(obj.pin1);
            [x2,y2,Dx2,Dy2,Hx2,Hy2]=obj.rigidBody2.rigid_body_position_derivatives(obj.pin2);
            
            
            A=zeros(2,numLagrange);
             %the A matrix is built from the gradients of the x and y
            %coordinates of the material point that we care about, where we
            %place those gradients in their correct indices in the A matrix
            %using the assigned global labels for those generalized coordinates
            A(1,obj.rigidBody1.coord_index)=Dx1;
            A(2,obj.rigidBody1.coord_index)=Dy1;
            A(1,obj.rigidBody2.coord_index)=-Dx2;
            A(2,obj.rigidBody2.coord_index)=-Dy2;
            
            
            %given the current state (velocity and position) of the rigid
            %body in question, compute the quadratic terms with respect to
            %velocity for both the x and y coordinates
            [v1,omega1]=obj.rigidBody1.get_v_and_omega();
            V1=[v1;omega1];
            
            [v2,omega2]=obj.rigidBody2.get_v_and_omega();
            V2=[v2;omega2];
            
            B(1)=-V1'*Hx1*V1+V2'*Hx2*V2;
            B(2)=-V1'*Hy1*V1+V2'*Hy2*V2;
        end
        
    end
    
    
    
end