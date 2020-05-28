classdef PolygonMath
   
    methods(Static)
        %this functions outputs the angle of vector v
        %with respect to the horizontal
        function theta=v_to_theta(v)
            s=size(v);
            theta=imag(log(v(1,:)+1i*v(2,:)));
        end
        
        %this function creates a counterclockwise rotation matrix
        %for some angle theta
        function rotmat_out=theta_to_rotmat(theta)
            rotmat_out=[cos(theta),-sin(theta);sin(theta),cos(theta)];
        end
        
        %this function maps point pin on some rigid body B
        %to their new locations
        %pout, after the rigid body has translated 
        %given by the position of the origin of the rigid body
        %in the world frame 'position', and it's rotation 'theta'
        function pout=rigid_body_position(position,theta,pin)
            s=size(pin);
            rot_matf=theta_to_rotmat(theta);
            pout=rot_matf*(pin)+repmat(position,[1,s(2)]);
        end
        
        
        %this function maps point pin on some rigid body B
        %to it's new location
        %pout, after the rigid body has translated 
        %given by the position of the origin of the rigid body
        %in the world frame 'position', and it's rotation 'theta'
        function vout=rigid_body_velocity(velocity,theta,omega,pin)
            s=size(pin);
            Q=[0,-1;1,0];
            rot_matf=theta_to_rotmat(theta);
            
            vout=omega*Q*rot_matf*(pin)+repmat(velocity,[1,s(2)]);
        end
        
        %this function maps point pin on some rigid body B
        %to it's new location
        %pout, after the rigid body has translated 
        %given by the position of the origin of the rigid body
        %in the world frame 'position', and it's rotation 'theta'
        function aout=rigid_body_acceleration(acceleration,theta,omega,alpha,pin)
            s=size(pin);
            Q=[0,-1;1,0];
            rot_matf=theta_to_rotmat(theta);
            aout=(omega^2)*Q*Q*rot_matf*(pin)+alpha*Q*rot_matf*(pin)+repmat(acceleration,[1,s(2)]);
        end

%         %this function maps point pin on some rigid body B
%         %to it's new location
%         %pout, after the rigid body has translated 
%         %given by the position of the origin of the rigid body
%         %in the world frame 'position', and it's rotation 'theta'
%         function vout=rigid_body_jacobian(velocity,theta,omega,pin)
%             s=size(pin);
%             rot_matf=theta_to_rotmat(theta);
% %             pout=rot_matf*(pin)+repmat(position,[1,s(2)]);
%         end
    end
end