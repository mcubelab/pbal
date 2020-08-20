
classdef PolygonConstraint < handle
    
    properties
        rigidBody1; %first rigid body object used by the constraint
        rigidBody2; %second rigid body object used by constraint (may not be used)
        
        pin1; %material point referenced on the first rigid body (may not be used) (in body frame)
        pin2; %material point referenced on the second rigid body (may not be used) (in body frame)
    
        normal_vector_Body1; %normal vector referenced on body 1 used for sliding contact
        normal_vector_Body2; %normal vector referenced on body 2 used for sliding contact
        normal_vector_World; %normal vector referenced in world frame used for sliding contact
        
        pout; %point used in the world frame
    
        constraint_draw; %plot of the constraint
        
        constraint_index; %label assigned to this constraint
        num_lagrange_multipliers; %number of lagrange multipliers associated with this constraint (usually 1 or 2)
        
        lagrange_multiplier_index; %labels assigned to the lagrange multipliers
        %takes the form of just one integer, or a sequence of consecutive
        %integers
        
        lagrange_multiplierValues;
        
        ConstraintType; %color/type of constraint, just an integer where 0 corresponds to nothing
        %1 sticking contact with one rigid body, and 1 point in world frame
        %2 sticking contact between two rigid bodies
        %3 sliding contact with one rigid body, and 1 point in WF
        %4 sliding contact with one rigid body, and 1 plane in WF
        %5 sliding contact between two rigid bodies 
        %(first one is point, second is plane)
        
    end
    
    methods
        %initializes blank object
        function obj=PolygonConstraint()
            obj.ConstraintType=0;
            obj.num_lagrange_multipliers=0;
        end
        
        %Creates the plot used to represent the kinematic constraint for the first time
        function initialize_visualization(obj)
            %call the associated initialize_visualization function for each type of
            %constraint
            if obj.ConstraintType==0
                disp('Error: Polygon Constraint Never Initialized');
            end
            
            if obj.ConstraintType==1
                obj.initialize_visualizationStickingContactOneBody();
            end
            
            if obj.ConstraintType==2
                obj.initialize_visualizationStickingContactTwoBodies();
            end
            
            if obj.ConstraintType==3
                
            end
            
            if obj.ConstraintType==4
                
            end
            
            if obj.ConstraintType==5
                
            end
        end
        
        %Creates the plot used to represent the kinematic constraint for the first time
        %this is specifically for sticking contact with one rigid body, and 1 point in world frame
        function initialize_visualizationStickingContactOneBody(obj)
             pout1=obj.rigidBody1.rigid_body_position(obj.pin1);
            
             obj.constraint_draw=line(...
                'XData',[obj.pout(1),pout1(1)],...
                'YData',[obj.pout(2),pout1(2)],...
                'color','k','linewidth',1,'Marker','o',...
                'Markerfacecolor','r','Markeredgecolor','r','markersize',2);
        end
        
        %Creates the plot used to represent the kinematic constraint for the first time
        %this is specifically for sticking contact between two rigid bodies
        function initialize_visualizationStickingContactTwoBodies(obj)
            pout1=obj.rigidBody1.rigid_body_position(obj.pin1);
            pout2=obj.rigidBody2.rigid_body_position(obj.pin2);
            
            obj.constraint_draw=line(...
                'XData',[pout1(1),pout2(1)],...
                'YData',[pout1(2),pout2(2)],...
                'color','k','linewidth',1,'Marker','o',...
                'Markerfacecolor','r','Markeredgecolor','r','markersize',2);
        end
        
        %Updates the plot of the constraint given the current state of the system
        function update_visualization(obj)
            %call the associated update_visualization function for each type of
            %constraint
            if obj.ConstraintType==0
                disp('Error: Polygon Constraint Never Initialized');
            end
            
            if obj.ConstraintType==1
                obj.update_visualizationStickingContactOneBody();
            end
            
            if obj.ConstraintType==2
                obj.update_visualizationStickingContactTwoBodies();
            end
            
            if obj.ConstraintType==3
                
            end
            
            if obj.ConstraintType==4
                
            end
            
            if obj.ConstraintType==5
                
            end
        end
        
        function setMultipliers(obj,lagrange_multiplierValues)
            obj.lagrange_multiplierValues=lagrange_multiplierValues;
        end
        
        function LagrangeOut = getMultipliers(obj)
            LagrangeOut=obj.lagrange_multiplierValues;
        end
        
        %Updates the plot of the constraint given the current state of the system
        %this is specifically for sticking contact with one rigid body, and 1 point in world frame
        function update_visualizationStickingContactOneBody(obj)
            pout1=obj.rigidBody1.rigid_body_position(obj.pin1);
            
            set(obj.constraint_draw,'XData',[obj.pout(1),pout1(1)],...
                                    'YData',[obj.pout(2),pout1(2)]);
        end
        
        %Updates the plot of the constraint given the current state of the system
        %this is specifically for sticking contact between two rigid bodies
        function update_visualizationStickingContactTwoBodies(obj)
            pout1=obj.rigidBody1.rigid_body_position(obj.pin1);
            pout2=obj.rigidBody2.rigid_body_position(obj.pin2);
            
            set(obj.constraint_draw,'XData',[pout1(1),pout2(1)],...
                                    'YData',[pout1(2),pout2(2)]);
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
        
        %stores information associated with a sliding contact between a
        %rigid body and a point in the world frame
        function SlidingContactOneBodyWorldPoint(obj,Body1,pin1,pout,normal_vector)
            obj.rigidBody1=Body1;
            obj.pin1=pin1;
            obj.pout=pout;
            obj.normal_vector_Body1=normal_vector;
            
            %assigns the sliding constraint label with 1 rigid body
            %sliding against 1 point in the world frame
            obj.ConstraintType=3; 
            
            obj.num_lagrange_multipliers=1; %x and y constriants -> 2 multipliers
        end
        
        function SlidingContactOneBodyWorldLine(obj,Body1,pin1,pout,normal_vector)
            obj.rigidBody1=Body1;
            obj.pin1=pin1;
            obj.pout=pout;
            obj.normal_vector_World=normal_vector;
            
            %assigns the sliding constraint label with 1 rigid body
            %sliding against 1 point in the world frame
            obj.ConstraintType=4; %assigns the sliding constraint label with 1 rigid body
            
            obj.num_lagrange_multipliers=1; %x and y constriants -> 2 multipliers
        end
        
        %5 sliding contact between two rigid bodies 
        %(first one is point, second is plane)
        function SlidingContactTwoBodies(obj,Body1,pin1,Body2,pin2,normal_vector)
            obj.rigidBody1=Body1;
            obj.pin1=pin1;
            obj.rigidBody2=Body2;
            obj.pin2=pin2;
            obj.normal_vector_Body2=normal_vector;
            
            obj.ConstraintType=5; %assigns the sliding constraint label with 1 rigid body
            
            obj.num_lagrange_multipliers=1; %x and y constriants -> 2 multipliers
        end
        
        %this builds the linear equation associated with the acceleration
        %form of the kinematic constraints, i.e
        %A(q,dq/dt)*d^2q/dt^2=B(q,dq/dt)
        %where q is the vector of generalized coordinates, B is a vector
        %associated with the quadratic velocity terms that appear when the
        %constraint is differentiated twice with respect to time
        %numCoords is the total number of generalized coords in the
        %simulation environment, which is needed to determine width of the
        %A matrix!
        function [A,B,ConstraintError] = generateBlock(obj,numCoords)
            
            %call the associated generateBlock function for each type of
            %constraint
            if obj.ConstraintType==0
                disp('Error: Polygon Constraint Never Initialized');
            end
            
            if obj.ConstraintType==1
                [A,B,ConstraintError] = obj.generateBlockStickingContactOneBody(numCoords);
            end
            
            if obj.ConstraintType==2
                [A,B,ConstraintError] = obj.generateBlockStickingContactTwoBodies(numCoords);
            end
        end
        
        
        %this builds the linear equation associated with the acceleration
        %form of the kinematic constraints, i.e
        %A(q,dq/dt)*d^2q/dt^2=B(q,dq/dt)
        %where q is the vector of generalized coordinates, B is a vector
        %associated with the quadratic velocity terms that appear when the
        %constraint is differentiated twice with respect to time
        %numCoords is the total number of generalized coords in the
        %simulation environment, which is needed to determine width of the
        %A matrix!
        
        %this is specifically for sticking contact with one rigid body, and 1 point in world frame
        function [A,B,ConstraintError] = generateBlockStickingContactOneBody(obj,numCoords)
            [x1,y1,Dx1,Dy1,Hx1,Hy1]=obj.rigidBody1.rigid_body_position_derivatives(obj.pin1);
           
            A=zeros(2,numCoords);
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
            
            ConstraintError=[x1-obj.pout(1);y1-obj.pout(2)];
        end
        
        %this builds the linear equation associated with the acceleration
        %form of the kinematic constraints, i.e
        %A(q,dq/dt)*d^2q/dt^2=B(q,dq/dt)
        %where q is the vector of generalized coordinates, B is a vector
        %associated with the quadratic velocity terms that appear when the
        %constraint is differentiated twice with respect to time
        %numCoords is the total number of lagrange multipliers in the
        %simulation environment, which is needed to determine width of the
        %A matrix!
        
        %this is specifically for sticking contact between two rigid bodies
        function [A,B,ConstraintError] = generateBlockStickingContactTwoBodies(obj,numCoords)
            [x1,y1,Dx1,Dy1,Hx1,Hy1]=obj.rigidBody1.rigid_body_position_derivatives(obj.pin1);
            [x2,y2,Dx2,Dy2,Hx2,Hy2]=obj.rigidBody2.rigid_body_position_derivatives(obj.pin2);
            
            
            A=zeros(2,numCoords);
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
            
            ConstraintError=[x1-x2;y1-y2];
        end
        
        
    end
    
    
    
end