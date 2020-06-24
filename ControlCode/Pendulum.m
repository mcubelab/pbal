classdef Pendulum
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        m;  % mass (kg)
        g;  % acceleration due to gravity (kg/m^2)
        l;  % pendulum length (m)
        h;  % time-step
        b;  % damping
        
        t_m % maximum torque (N * m)
        
    end
    
    methods
        
        function obj = Pendulum(params)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.m = params.m;
            obj.l = params.l;
            obj.g = 9.81;
            
            obj.t_m = 0.2;
            obj.h = 0.01;
            obj.b = 0.05; 
        end
        
        
        function [xkp1, uk] =  dynamics_solve(obj, xk, uk)
            
            [Adyn, bdyn] = obj.dynamics_const(xk);
            [Ax, bx] = obj.build_state_const();
            [Au, bu] = obj.build_input_const();
            
            
            z = fmincon(@(z) 0, [xk; uk], Au, bu, [Adyn; Ax], [bdyn; bx]); 
            
            xkp1 = z(1:6);
            uk = z(7:9); 
       
            
        end
        
        function [A, B] = linearize_forward_dyn(obj, x0, u0)
           
            delta = 1e-6;
            
            Ix = eye(6);
            A = zeros(6, 6);
            for i = 1:6
               xkp1_p = obj.forward_dyn_euler(x0 + delta * Ix(:, i), u0);
               xkp1_m = obj.forward_dyn_euler(x0 - delta * Ix(:, i), u0);
               A(:, i) = (xkp1_p - xkp1_m)/(2 * delta); 
            end
            
            Iu = eye(3);
            B = zeros(6, 3);
            for i = 1:3
               xkp1_p = obj.forward_dyn_euler(x0, u0  + delta * Iu(:, i));
               xkp1_m = obj.forward_dyn_euler(x0, u0  - delta * Iu(:, i));
               B(:, i) = (xkp1_p - xkp1_m)/(2 * delta); 
            end                
            
        end
        
        function xkp1 = forward_dyn_euler(obj, xk, uk)
                
            qk = xk(1:3);
            qkd = xk(4:6);
                        
            M = obj.build_mass_matrix(qk);
            c = obj.build_coriolis_and_potential(qk, qkd);
            B  = obj.build_input_matrix();
            
            xkp1 = xk + obj.h * [qkd; -M\(c + B * uk)]; 
            
        end
            
        
        function [A, b] = dynamics_const(obj, xk)
            
            qk = xk(1:3);
            qkd = xk(4:6);
            
            M = obj.build_mass_matrix(qk);
            c = obj.build_coriolis_and_potential(qk, qkd);
            B  = obj.build_input_matrix();
            
            A = [eye(6), [zeros(3); obj.h * (M\B)]];
            b = xk + obj.h*[qkd; (-M\c)];
            
        end
        
        function M = build_mass_matrix(obj, qk)
            
            tht = qk(3);
            
            M = [obj.m, 0, 0 *obj.m*obj.l*cos(tht);
                0, obj.m, 0 * obj.m*obj.l*sin(tht);
                0 * obj.m*obj.l*cos(tht), 0 *obj.m*obj.l*sin(tht), obj.m * obj.l^2];
            
        end
        
        function c = build_coriolis_and_potential(obj, qk, qkd)
            
            tht = qk(3);
            
            xd = qkd(1);
            yd = qkd(2);
            thtd = qkd(3);
            
            c = [-obj.m * obj.l * thtd^2 * sin(tht);
                obj.m * obj.l * thtd^2 * cos(tht) - obj.m * obj.g;
                obj.m * obj.l * (thtd * (yd * cos(tht) - xd * sin(tht)) + obj.g * sin(tht))];
        
            c = c + obj.b*qkd; 
        end
        
        
        
        function B = build_input_matrix(obj)
            
            B = eye(3);
        end
        
        function [Au, bu] = build_input_const(obj)
            Au = [zeros(1,6), 0, 0, -1; zeros(1,6), 0, 0, 1];
            bu = [obj.t_m; obj.t_m];
        end
        
        function [Ax, bx] = build_state_const(obj)
            Ax = [1, 0, 0, zeros(1, 6); 0, 1, 0, zeros(1,6); 
                0, 0, 0, 1, zeros(1, 5); 0, 0, 0, 0, 1, zeros(1,4)];
            bx = [0; 0; 0; 0];
        end
    end
end

