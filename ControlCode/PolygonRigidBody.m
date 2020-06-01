classdef PolygonRigidBody < handle
    properties
        Mass; %Mass of the rigid body
        MoI; %Moment of inertia of the rigid body w/respect to the center of mass
        RadiusGyration; %Radius of gyration of the rigid body
        LocalPolygon; %The MyPolygon instance contained by the rigid body
                       %which describes the kinematics of the system
        
        LinearMomentum;  %Linear momentum of the rigid body
        AngularMomentumCOM; %Angluar memomentum of the rigid body taken
                            %with respect to the center of mass
        
    end
    
    methods
        function obj=PolygonRigidBody(plist, r_cm_body, m, I)
            obj.Mass=m;
            obj.MoI=I;
            obj.LocalPolygon=MyPolygon(plist, r_cm_body);
        end
        
        %Creates the plots used to represent the polygon for the first time
        function initialize_visualization(obj)
            obj.LocalPolygon.initialize_visualization();
        end
        
        %Updates the plot given the current state of the polygon
        function update_visualization(obj)
            obj.LocalPolygon.update_visualization();
        end
        
        %this function updates the location of the polygon vertices
        function update_plist(obj,plist)
            obj.LocalPolygon.update_plist(plist);
        end
        
        %this function updates the location of the center of mass
        %in the body frame!!!!
        function update_cm_location(obj,r_cm_body)
            obj.LocalPolygon.update_cm_location(r_cm_body);
        end
        
        %this function sets all system state values to zero
        function base_state(obj)
            obj.LocalPolygon.base_state();
        end
        
        %this function sets the state of the rigid body
        function set_state(obj,p,v,a,theta,omega,alpha)
            obj.LocalPolygon.set_state(p,v,a,theta,omega,alpha);
            
            obj.update_momentum();
        end
        
        %this function gets the state of the rigid body
        function [p,v,a,theta,omega,alpha]=get_state(obj)
            p=obj.LocalPolygon.position;
            v=obj.LocalPolygon.velocity;
            a=obj.LocalPolygon.acceleration;
            theta=obj.LocalPolygon.theta;
            omega=obj.LocalPolygon.omega;
            alpha=obj.LocalPolygon.alpha;
        end
        
        %Generates the current linear and angular momentum of the system
        %In the world frame, given the current linear and angular
        %velocities
        function update_momentum(obj)
            obj.LinearMomentum=obj.LocalPolygon.velocity*obj.Mass;
            obj.AngularMomentumCOM=obj.LocalPolygon.omega*obj.MoI;
        end
        
        function [LM,AM]=get_momentum(obj)
            LM=obj.LinearMomentum;
            AM=obj.AngularMomentumCOM;
        end
        
        %this function sets the value of postion and theta
        function set_p_and_theta(obj,p,theta)
            obj.LocalPolygon.set_p_and_theta(p,theta);
        end
        
        %this function gets the value of postion and theta
        function [p,theta]=get_p_and_theta(obj)
            p=obj.LocalPolygon.position;
            theta=obj.LocalPolygon.theta;
        end
        
        %this function sets the value of velocity and omega
        function set_v_and_omega(obj,v,omega)
            obj.LocalPolygon.set_v_and_omega(v,omega);
            obj.update_momentum();
        end
        
        %this function gets the value of velocity and omega
        function [v,omega]=get_v_and_omega(obj)
            v=obj.LocalPolygon.velocity;
            omega=obj.LocalPolygon.omega;
        end
        
        %this function sets the value of acceleration and alpha
        function set_a_and_alpha(obj,a,alpha)
            obj.LocalPolygon.set_a_and_alpha(a,alpha);
        end
        
        %this function gets the value of acceleration alpha
        function [a,alpha]=get_a_and_alpha(obj)
            a=obj.LocalPolygon.acceleration;
            alpha=obj.LocalPolygon.alpha;
        end
        


    end
end