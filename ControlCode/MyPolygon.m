%This class defines the properties and behaviour of a polygonal rigid body
%Given the state of the rigid body (pos/vel/acc/theta/omega/alpha)
%We want to be able to do things like plot the polygon, figure out the
%positions/velocities/accelerations of its vertices
%determine the normal directions of its edges, and more
%this class will also assist in writing linear constraints to describe the
%kinematics of contact
classdef MyPolygon < handle
    
    %n_points is the number of vertice in the polygon
    %plist is the set of vertices of the polygon in its base state
    %r_cm_body is the position of the center of mass (body frame)
    %frame_rotation is the rotation matrix mapping the base orientation
    %to the current orientation
    %position is the current position of the frame
    %velocity is the current velocity of the frame
    %acceleration is the current acceleration of the frame
    %theta is the angle of the rigid body
    %omega is the angular velocity of the rigid body
    %alpha is the angular acceleration of the rigid body
    %body_draw is the plot of the rigid body
    %r_cm_world is the position of the center of mass (world frame)
    %v_cm_world is the velocity of the center of mass of (world frame)
    %a_cm_world is the acceleartion of the center of mass (world frame)
    %plist_current is the vertices of the polygon in its current state
    properties
        plist;
        
        frame_rotation;
        position;
        velocity;
        acceleration;
        theta;
        omega;
        alpha;
        body_draw;
        n_points;
        
        r_cm_body;
        
        r_cm_world;
        v_cm_world;
        a_cm_world;
        
        plist_current;
        vlist_current;
        alist_current;
    end
        
    methods
        %this function initializes the object
        %body_curve is the parametric curve representing the body
        %n_points is the number of points used to draw the body
        function obj= ButterflyRigidBody(plist, r_cm_body)
            s=size(plist);
            
            obj.plist=plist;
            obj.r_cm_body=r_cm_body;
            obj.n_points=s(2);
            
            obj.base_state();
        end
        
        %this function updates the location of the polygon vertices
        function update_plist(obj,plist)
            obj.plist=plist;
        end
        
        %this function updates the location of the center of mass
        %in the body frame!!!!
        function update_cm_location(obj,r_cm_body)
            obj.r_cm_body=r_cm_body;
        end
        
        %this function sets all system state values to zero
        function base_state(obj)
            obj.position=[0;0];
            obj.velocity=[0;0];
            obj.acceleration=[0;0];
            obj.theta=0;
            obj.omega=0;
            obj.alpha=0;
            obj.frame_rotation=PolygonMath.theta_to_rotmat(0);
        end
             
        %this function sets the state of the rigid body
        function set_state(obj,p,v,a,theta,omega,alpha)
            obj.position=p;
            obj.velocity=v;
            obj.acceleration=a;
            obj.theta=theta;
            obj.omega=omega;
            obj.alpha=alpha;
            obj.frame_rotation=PolygonMath.theta_to_rotmat(obj.theta);
        end
        
        %this function gets the state of the rigid body
        function [p,v,a,theta,omega,alpha]=get_state(obj)
            p=obj.position;
            v=obj.velocity;
            a=obj.acceleration;
            theta=obj.theta;
            omega=obj.omega;
            alpha=obj.alpha;
        end
        
        %this function sets the value of postion and theta
        function set_p_and_theta(obj,p,theta)
            obj.position=p;
            obj.theta=theta;
            obj.frame_rotation=PolygonMath.theta_to_rotmat(obj.theta);
        end
        
        %this function gets the value of postion and theta
        function [p,theta]=get_p_and_theta(obj)
            p=obj.position;
            theta=obj.theta;
        end
        
        %this function sets the value of velocity and omega
        function set_v_and_omega(obj,v,omega)
            obj.velocity=v;
            obj.omega=omega;
        end
        
        %this function gets the value of velocity and omega
        function [v,omega]=get_v_and_omega(obj)
            v=obj.velocity;
            omega=obj.omega;
        end
        
        %this function sets the value of acceleration and alpha
        function set_a_and_alpha(obj,a,alpha)
            obj.acceleration=a;
            obj.alpha=alpha;
        end
        
        %this function gets the value of acceleration alpha
        function [a,alpha]=get_a_and_alpha(obj)
            a=obj.acceleration;
            alpha=obj.alpha;
        end
        
        %this function updates plist_current for a new position
        %as well as the center of mass location in the world frame
        function update_current_points(obj)
            obj.plist_current=PolygonMath.solid_body_position...
            (obj.position,obj.theta,obj.plist);
        
            obj.r_cm_world=PolygonMath.solid_body_position...
            (obj.position,obj.theta,obj.r_cm_body);
        end
        
        %this function updates plist_current for a new position
        %as well as the center of mass velocity in the world frame
        function update_current_velocities(obj)
            obj.vlist_current=PolygonMath.solid_body_velocity...
            (obj.velocity,obj.theta,obj.omega,obj.plist);
        
            obj.v_cm_world=PolygonMath.solid_body_velocity...
            (obj.velocity,obj.theta,obj.omega,obj.r_cm_body);
        end
        
        %this function updates plist_current for a new position
        %as well as the center of mass velocity in the world frame
        function update_current_accelerations(obj)
            obj.alist_current=PolygonMath.solid_body_acceleration...
            (obj.acceleration,obj.theta,obj.omega,obj.alpha,obj.plist);
        
            obj.a_cm_world=PolygonMath.solid_body_acceleration...
            (obj.acceleration,obj.theta,obj.omega,obj.alpha,obj.r_cm_body);
        end
    end
end