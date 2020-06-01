classdef PolygonContacts < handle
    
    properties
        nominal_position            % nominal position of contact [x, y, tht]
        contact_geom                % contact type: PointContact, LineContact, or PatchContact
        is_env                      % is the contact frame defined in the world (True) or body (False) frame
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
        function Gf = force_jacobian(cpos)
            
            rg = 1;     % TODO: what about rg?
            
            rx = cpos(1); ry = cpos(2); rz = cpos(3);
            
            Gf = [cos(rz), -sin(rz), 0;
                sin(rz), cos(rz), 0;
                rg*(rx * sin(rz) - ry * cos(rz)), rg*(rx * cos(rz) + ry * sin(rz)), 1];
            
        end
        
        % This function generates a jacobian that takes generalized velocities from
        % the frame located at the origin to the frame located at cpos0 - xk
        % Inputs:
        %    xk: [x, y, rg * tht] is the generalized configuration on the object
        % Outputs:
        %    Gv such that v_b = Gv * v_c, where v = [v_x, v_y, omega * rg]
        function Gv = velocity_jacobian(cpos)
            
            rg = 1;     % TODO: what about rg?
            
            rx = cpos(1); ry = cpos(2); rz = cpos(3);
            
            Gv = [cos(rz), sin(rz), (rx * sin(rz) - ry * cos(rz))/rg;
                -sin(rz), cos(rz), (rx * cos(rz) + ry * sin(rz))/rg;
                0, 0, 1];
        end
        
        % This function generates the normal vectors associated with the hyperplanes
        % that define the polytope of wrenches this contact can exert in the body frame
        % Inputs:
        %    xk: [x, y, rg * tht] is the generalized configuration on the object
        % Outputs:
        %    normals: each row is the inward pointing unit-normal for a
        %    hyperplane
        function [n_eq, n_iq] = wrench_polytope_body_frame(xk)
            
            cpos = obj.get_contact_position_body_frame(xk);
            Gf_contact_body = obj.force_jacobian(cpos);
            
            [neq_c, niq_c] = obj.wrench_polytope_contact_frame();
            n_eq = Gf_contact_body * neq_c;
            n_iq = Gf_contact_body * niq_c;
        end
        
        % This function generates the normal vectors associated with the hyperplanes
        % that define the polytope of wrenches this contact can exert in the world frame
        % Inputs:
        %    xk: [x, y, rg * tht] is the generalized configuration on the object
        % Outputs:
        %    normals: each row is the inward pointing unit-normal for a
        %    hyperplane
        function [n_eq, n_iq] =  wrench_polytope_world_frame(xk)
            cpos = obj.get_contact_position_world_frame(xk);
            Gf_contact_world = obj.force_jacobian(cpos);
            
            [neq_c, niq_c] = obj.wrench_polytope_contact_frame();
            n_eq = Gf_contact_world * neq_c;
            n_iq = Gf_contact_world * niq_c;
        end
        
        % This function generates the linear constraints on kinematically feasible
        % velocities in the body frame
        % Inputs:
        %    xk: [x, y, rg * tht] is the generalized configuration on the object
        % Outputs:
        %    A, b: such that A * vb = b
        function [A, b] = velocity_constraint_body_frame(xk)
            cpos = obj.get_contact_position_body_frame(xk);
            Gv_contact_body = obj.velocity_jacobian(cpos);
            
            [A_c, b] = obj.velocity_constraint_contact_frame();
            A = Gv_contact_body * A_c;
        end
        
        % This function generates the linear constraints on kinematically feasible
        % velocities in the world frame
        % Inputs:
        %    xk: [x, y, rg * tht] is the generalized configuration on the object
        % Outputs:
        %    A, b: such that A * vw = b
        function [A, b] = velocity_constraint_world_frame(xk)
            
            cpos = obj.get_contact_position_world_frame(xk);
            Gv_contact_world = obj.velocity_jacobian(cpos);
            
            [A_c, b] = obj.velocity_constraint_contact_frame();
            A = Gv_contact_world * A_c;
        end
        
        % Returns the position and orientation of the contact frame in the
        % body frame
        % Inputs:
        %    xk: [x, y, rg * tht] is the generalized configuration on the object
        % Outputs:
        %    cpos: [x, y, rg* tht] is position and orientation of the contact
        %    frame relative to the COM in the body frame
        function cpos = get_contact_position_body_frame(xk)
            % TODO: deal with sliding here.
            % TODO: what about r_g and angle wrapping?
            
            if obj.is_env
                cpos = obj.nominal_position - [0, 0, 1]*xk;
            else
                cpos = obj.nominal_position;
            end
        end
        
        % Returns the position and orientation of the contact frame in the
        % world frame
        % Inputs:
        %    xk: [x, y, rg * tht] is the generalized configuration on the object
        % Outputs:
        %    cpos: [x, y, rg* tht] is position and orientation of the contact
        %    frame relative to the COM in the world frame
        function cpos = get_contact_position_world_frame(xk)
            % TODO: deal with sliding here.
            % TODO: what about r_g and angle wrapping?
            
            if obj.is_env
                cpos = obj.nominal_position;
            else
                cpos = obj.nominal_position -  - [0, 0, 1]*xk; % TODO: what about r_g?
            end
        end
        
    end
    
    methods(Static)
        
        % This is a wrapper that calls the appropriate function based on contact
        % geometry to generate to the normal vectors associated with the hyperplane
        % that define the polytope of wrenches this contact can exert in the contact frame
        function[n_eq, ni_q] = wrench_polytope_contact_frame()
            [n_eq, ni_q] = obj.contact_geom.wrench_polytope_contact_frame();
        end
        
        % This is a wrapper that calls the appropriate function based on contact
        % geometry to generate kinematic constraints on velocities in the contact frame
        function[Aeq, beq, Aiq, biq] = velocity_constraint_contact_frame()
            [Aeq, beq, Aiq, biq] = obj.contact_geom.wrench_polytope_contact_frame();
        end
        
    end
end


