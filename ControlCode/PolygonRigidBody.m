classdef PolygonRigidBody < handle
    properties
        rigid_body_index; %label of this rigid body within the simulation environments
        coord_index; %labels of the generalized coords within the sim environemt
        %coord_index is a list of 3 consecutive numbers 1,2,3 or 4,5,6 etc.
        %corresponding x,y, theta
        
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
        
        %Calls the rigid body derivative function, using the current state
        %of this polygon rigid body
        function [x,y,Dx,Dy,Hx,Hy]=rigid_body_position_derivatives(obj,pin)
            [x,y,Dx,Dy,Hx,Hy]=PolygonMath.rigid_body_position_derivatives(obj.LocalPolygon.position,obj.LocalPolygon.theta,pin);
        end
        
        %Calls the rigid body derivative function, using the current state
        %of this polygon rigid body, assuming that you are looking for the
        %center of mass information
        function [x,y,Dx,Dy,Hx,Hy]=rigid_body_CM_derivatives(obj)
            [x,y,Dx,Dy,Hx,Hy]=PolygonMath.rigid_body_position_derivatives(obj.LocalPolygon.position,obj.LocalPolygon.theta,obj.LocalPolygon.r_cm_body);
        end
        
        %This function outputs the associated matrix and vector that show
        %up in the lagrange equation for this rigid body. Specifically, the
        %lagrange equations will have the form of:
        %M*AccelVector = V + GeneralizedForces
        %Where AccelVector = [d2x/dt2;d2y/dt2;alpha]
        function [M,V] = LagrangeVM(obj)
%             [~,~,Dx_cm,Dy_cm,Hx_cm,Hy_cm]=PolygonMath.rigid_body_position_derivatives(obj.LocalPolygon.position,obj.LocalPolygon.theta,obj.LocalPolygon.r_cm_body);
%             [~,~,Dx_cm,Dy_cm,Hx_cm,Hy_cm]=obj.rigid_body_position_derivatives(obj.LocalPolygon.r_cm_body);
            [~,~,Dx_cm,Dy_cm,Hx_cm,Hy_cm]=obj.rigid_body_CM_derivatives();
            
            v_cm=PolygonMath.rigid_body_velocity(obj.LocalPolygon.velocity,obj.LocalPolygon.theta,obj.LocalPolygon.omega,obj.LocalPolygon.r_cm_body);
            M=obj.Mass*(Dx_cm')*Dx_cm+obj.Mass*(Dy_cm')*Dy_cm+obj.MoI*[0,0,0;0,0,0;0,0,1];
            
            Q=[0,-1;1,0];
            rot_matf=PolygonMath.theta_to_rotmat(obj.LocalPolygon.theta);
            
            dGeneralized_dt=[obj.LocalPolygon.velocity;obj.LocalPolygon.omega];
            M_temp=Hx_cm*dGeneralized_dt*Dx_cm+Hy_cm*dGeneralized_dt*Dy_cm;
            V1=obj.Mass*(M_temp+M_temp')*dGeneralized_dt;
            V2=obj.Mass*(v_cm')*(Q*Q*rot_matf*obj.LocalPolygon.r_cm_body*obj.LocalPolygon.omega);
            V=V2-V1;
        end
        
        %This function does a simple forward-euler update of the 
        %positions and velocities of the polygon, given some time step dt
        function EulerUpdate(obj,dt)
            %does an euler step on the associated polyon contained in the
            %rigid body
            obj.LocalPolygon.EulerUpdate(dt);
        end
        


    end
end