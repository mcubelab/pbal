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
    %plist should be a 2xN array
    %r_cm_body is the position of the center of mass (body frame)
    %frame_rotation is the rotation matrix mapping the base orientation
    %to the current orientation
    %position is the current position of the frame
    %velocity is the current velocity of the frame
    %acceleration is the current acceleration of the frame
    %theta is the angle of the rigid body
    %omega is the angular velocity of the rigid body
    %alpha is the angular acceleration of the rigid body
    %r_cm_world is the position of the center of mass (world frame)
    %v_cm_world is the velocity of the center of mass of (world frame)
    %a_cm_world is the acceleartion of the center of mass (world frame)
    %plist_current is the vertices of the polygon in its current state
    %normal_list is the list of unit normals for each of the edges of the 
    %polygon (in the body frame). It is defined so that the normals will be
    %outward facing if the vertices are listed in counterclockwise order
    %normal_list_current is the list of unit normal for each edge in the
    %world frame
    %tangent_list is the list of unit tangents for each of the edges of the
    %polygon (in the body frame). Tangent i points from vertex i to i+1
    %tangent_list_current is the list of unit tangents for each of the
    %edges of the polygon (in the world frame). Tangent i points from
    %vertex i to i+1
    %body_draw is the plot of the vertices and edges of the polygon
    %body_draw_COM is the plot of the COM of the polygon
    properties
        plist; 
   
        frame_rotation;
        position;
        velocity;
        acceleration;
        theta;
        omega;
        alpha;
        n_points;
        
        
        body_draw;
        body_draw_COM;
        
        %If two consecutive vertices in plist are the same
        %then the corresponding normal and tangents are 0!
        %I also assume that the polygon is closed, so that the first vertex
        %is also the n+1'th vertex (n total vertices)
        normal_list;
        tangent_list;
        
        r_cm_body;
        
        r_cm_world;
        v_cm_world;
        a_cm_world;
        
        plist_current;
        vlist_current;
        alist_current;
        
        
        normal_list_current;
        tangent_list_current;
    end
        
    methods
        %this function initializes the object
        %body_curve is the parametric curve representing the body
        %n_points is the number of points used to draw the body
        function obj= MyPolygon(plist, r_cm_body)
            s=size(plist);
            
            obj.update_plist(plist);
            obj.r_cm_body=r_cm_body;
            obj.n_points=s(2);
            
            obj.base_state();
            obj.update_current_points();
            
        end
        
        %Creates the plots used to represent the polygon for the first time
        function initialize_visualization(obj)
            obj.body_draw=line(...
                'XData',[obj.plist_current(1,:),obj.plist_current(1,1)],...
                'YData',[obj.plist_current(2,:),obj.plist_current(2,1)],...
                'color','b','linewidth',2,'Marker','o',...
                'Markerfacecolor','k','Markeredgecolor','k','markersize',3);
            
            obj.body_draw_COM=line(...
                'XData',obj.r_cm_world(1,1),...
                'YData',obj.r_cm_world(2,1),...
                'Marker','o',...
                'Markerfacecolor','r','Markeredgecolor','r','markersize',6);
        end
        
        %Updates the plot given the current state of the polygon
        function update_visualization(obj)
            set(obj.body_draw,...
                'XData',[obj.plist_current(1,:),obj.plist_current(1,1)],...
                'YData',[obj.plist_current(2,:),obj.plist_current(2,1)]);
            set(obj.body_draw_COM,...
                'XData',obj.r_cm_world(1,1),...
                'YData',obj.r_cm_world(2,1));
        end
        
        %this function updates the location of the polygon vertices
        function update_plist(obj,plist)
            obj.plist=plist;
            obj.update_body_frame_normals_and_tangents();
        end
        
        %this function updates the directions of the tangents and normals 
        %of the polygon in the body frame
        %given the list of vertices of the polygon.
        function update_body_frame_normals_and_tangents(obj)
            
            temp_tangents=diff([obj.plist,obj.plist(:,1)],1,2);
            tangent_lengths=sqrt(sum(temp_tangents.^2,2));
            
            for count=1:length(tangent_lengths)
                if(tangent_lengths(count)==0)
                    temp_tangents(:,count)=temp_tangents(:,count)*0;
                else
                    temp_tangents(:,count)=temp_tangents(:,count)/tangent_lengths(count);
                end
            end
            
            obj.tangent_list=temp_tangents;
            
            Q=[0,-1;1,0];
            
            obj.normal_list=Q*obj.tangent_list;
        end
        
        %this function updates the directions of the tangents and normals 
        %of the polygon in the world frame
        %given the list of vertices of the polygon, and its orientation
        %in the world frame
        function update_world_frame_normals_and_tangents(obj)
            obj.tangent_list_current=PolygonMath.rigid_body_position...
            ([0;0],obj.theta,obj.tangent_list);
        
            obj.normal_list_current=PolygonMath.rigid_body_position...
            ([0;0],obj.theta,obj.normal_list);
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
            
            obj.update_current_points();
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
            
            obj.update_current_points();
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
            obj.plist_current=PolygonMath.rigid_body_position...
            (obj.position,obj.theta,obj.plist);
        
            obj.r_cm_world=PolygonMath.rigid_body_position...
            (obj.position,obj.theta,obj.r_cm_body);
        
            obj.update_world_frame_normals_and_tangents();
        end
        
        %this function updates plist_current for a new position
        %as well as the center of mass velocity in the world frame
        function update_current_velocities(obj)
            obj.vlist_current=PolygonMath.rigid_body_velocity...
            (obj.velocity,obj.theta,obj.omega,obj.plist);
        
            obj.v_cm_world=PolygonMath.rigid_body_velocity...
            (obj.velocity,obj.theta,obj.omega,obj.r_cm_body);
        end
        
        %this function updates plist_current for a new position
        %as well as the center of mass velocity in the world frame
        function update_current_accelerations(obj)
            obj.alist_current=PolygonMath.rigid_body_acceleration...
            (obj.acceleration,obj.theta,obj.omega,obj.alpha,obj.plist);
        
            obj.a_cm_world=PolygonMath.rigid_body_acceleration...
            (obj.acceleration,obj.theta,obj.omega,obj.alpha,obj.r_cm_body);
        end
    end
end