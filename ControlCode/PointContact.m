classdef PointContact < handle
    
    properties
        contact_mode                   % contact mode (stick, slide_1, slide_2, ..., slide_n)
        friction                       % friction: coefficient of friction
    end
    
    methods
        
        % contact_mode is either stick, slide_1, slide_2, ..., or slide_n
        % friction is the coefficient of friction
        function obj= PointContact(contact_mode, friction)
            
            obj.contact_mode = contact_mode;
            obj.friction = friction;
            
        end
        
    end
    
    methods(Static)
        
        % This function generates the normal vectors associated with the hyperplanes
        % that define the polytope of wrenches this contact can exert in the contact frame
        % Outputs:
        %    neq: each row is the unit-normal for a hyperplane
        %    niq: each row is unit-normal for the halfspace such that 
        %    niq_i^T * w >=0 enforce the constraints
        function [n_eq, n_iq] = wrench_polytope_contact_frame()
            
            gen_norm = sqrt(obj.friction^2 + 1);
            
            switch obj.contact_mode
                
                case "stick"
                    n_eq = [0, 0, 1];                        % fz = 0                    
                    n_iq = [obj.friction, 1, 0;              % ft >= -mu*fn
                        -obj.friction, 1, 0]/gen_norm;       % ft <= mu*fn
                    
                case "slide_1"                  % sliding positive
                    
                    n_eq = [0, 0, 1];                        % fz = 0                    
                    n_iq = [-obj.friction, -1, 0]/gen_norm;  % ft >= mu*fn     
                    
                case "slide_2"                  % sliding negative
                    
                    n_eq = [0, 0, 1];                        % fz = 0                    
                    n_iq = [obj.friction, -1, 0];            % ft <= -mu*fn                        
                    
                otherwise
                   error(['PointContact/wrench_polytope_contact_frame: ', ...
                        'incorrect contact mode'])
                    
            end
            
        end
        
        % This function generates the linear constraints on kinematically feasible
        % velocities in the contact frame
        % Outputs:
        %    Aeq, beq: such that Aeq * vc = beq
        %    Aiq, biq: such that Aiq * vc >= biq
        function [Aeq, beq, Aiq, biq] = velocity_constraint_contact_frame()
            
            switch obj.contact_mode
                case "stick"
                    Aeq = [1, 0, 0; 0, 1, 0];  % vn = vt = 0
                    beq = [0, 0];
                    
                    Aiq =[];
                    biq = [];
                    
                case "slide_1" % sliding positive
                    
                    Aeq = [1, 0, 0];           % vn = 0
                    beq = 0;
                        
                    Aiq =[0, 1, 0];            % vt >= 0
                    biq = 0;
                    
                case "slide_2" % sliding negative
                    Aeq = [1, 0, 0];          % vn = 0
                    beq = 0;
                        
                    Aiq =[0, -1, 0];          % vt <= 0
                    biq = 0;
                    
                otherwise
                    error(['PointContact/velocity_constraint_contact_frame: ', ...
                        'incorrect contact mode'])
            end
            
            
        end
        
        
        
    end
end


