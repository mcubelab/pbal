classdef PolygonContacts < handle
    
    properties    
        nominal_position            % nominal position of contact [x, y, tht]
        contact_geom                % contact type: PointContact, LineContact, or PatchContact
        is_env                      % is the contact frame defined in the world or body frame
    end
    
    methods 
        
    % this function initializes the object
    % is_env defines wether the nominal_position is the body or world frame
    % nominal position of the object
    % contact_geom is the contact geometry ("point", "line", or "patch")
    % contact_mode is the contact mode (stick, slide_1, slide_2, ...,
    % slide_n) 
    % friction is the coefficient of friction
    function obj= PolygonContacts(is_env, nominal_position, contact_geom, ...
            contact_mode, friction)

        obj.nominal_position = nominal_position;
        obj.is_env = is_env;
        
        switch contact_geom           
            case "point"
                obj.contact_geom = PointContact(contact_mode, friction);
            case "line"
                error('PolygonContacts/PolygonContacts: not implemented')          
            case "patch"
                error('PolygonContacts/PolygonContacts: not implemented')               
            otherwise
                error('PolygonContacts/PolygonContacts: incorrect contact type')            
        end
    end
    
    % This function generates a jacobian that takes generalized forces from
    % frame located at cpos0 - xk to the frame located at the origin. 
    % Inputs: 
    %    xk: [x, y, rg * tht] is the generalized configuration on the object
    % Outputs: 
    %    Gf such that f_c = Gf * f_b, where f = [f_x, f_y, tau/rg]        
    function Gf = force_jacobian(xk)
    end
    
    % This function generates a jacobian that takes generalized velocities from
    % the frame located at the origin to the frame located at cpos0 - xk
    % Inputs: 
    %    xk: [x, y, rg * tht] is the generalized configuration on the object
    % Outputs: 
    %    Gf such that v_b = Gf * v_c, where f = [r_x, r_y, omega * rg]    
    function Gv = velocity_jacobian(xk)  
    end
    
    % This function generates the normal vectors associated with the hyperplanes
    % that define the polytope of wrenches this contact can exert in the body frame
    % Inputs: 
    %    xk: [x, y, rg * tht] is the generalized configuration on the object
    % Outputs: 
    %    normals: each row is the inward pointing unit-normal for a
    %    hyperplane     
    function normals = wrench_polytope_body_frame(xk)
    end
    
    % This function generates the normal vectors associated with the hyperplanes
    % that define the polytope of wrenches this contact can exert in the world frame
    % Inputs: 
    %    xk: [x, y, rg * tht] is the generalized configuration on the object
    % Outputs: 
    %    normals: each row is the inward pointing unit-normal for a
    %    hyperplane        
    function normals =  wrench_polytope_world_frame(xk)
    end
    
    % This function generates the linear constraints on kinematically feasible
    % velocities in the body frame
    % Inputs: 
    %    xk: [x, y, rg * tht] is the generalized configuration on the object
    % Outputs: 
    %    A, b: such that A * vb = b 
    function [A, b] = velocity_constraint_body_frame(xk)
    end
       
    % This function generates the linear constraints on kinematically feasible
    % velocities in the world frame
    % Inputs: 
    %    xk: [x, y, rg * tht] is the generalized configuration on the object
    % Outputs: 
    %    A, b: such that A * vw = b 
    function [A, b] = velocity_constraint_world_frame(xk)
    end

        
    end
end


