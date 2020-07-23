classdef RigidBodyPendulum
    % A rod of mass (m) and length(l) pivoting about its top end in the
    % gravity plane
    
    properties
        
        % user specified
        m;   % mass (kg)
        l;   % pendulum length (m)
        t_m;  % control torque limit (N * m)
        b;   % damping coeff
        
        % fixed/derived
        g;              % acceleration due to gravity (kg/m^2)
        I;              % inertia about COM (kg^2 * m^2)
        fmincon_opt;    % options for fmincon
        
        % dimensions
        nq;                 % config dimension
        nv;                 % gen velocity dimension
        nx;                 % state dimension
        nu;                 % input dimension
        neq;                % # of equality const
        niq;                % # of inequality const
        
    end
    
    methods
        
        % initialize
        function obj = RigidBodyPendulum(params)
            
            obj.m = params.m;       % mass  (kg)
            obj.l = params.l;       % length (m)
            obj.t_m = params.t_m;   % control torque limit (N*m)
            obj.b = params.b;       % damping coefficient
            
            obj.g = params.g;                   % gravity (m/s^2)
            obj.I = obj.l^2 * obj.m^2/12;   % inertia (kg^2 * m^2)
            
            % dimensions
            obj.nq = 1;
            obj.nv = 1;
            obj.nx = obj.nq + obj.nv;
            obj.nu = 1;
            obj.neq = 1;
            obj.niq = 2 * obj.nu;
        end
        
        
        % continuous forward dynamics and first derivatives
        % xk = [qk; qkd]
        % [qkd, qkdd] = [qkd; M^{-1} * (B(q) * uk - c(qd))]
        function [f, df_dx, df_du] = dynamics(obj, xk, uk)
            
            % separate state
            qk = xk(1:obj.nq);
            qkd = xk(obj.nq + (1:obj.nv));
            
            f = [qkd; (uk - obj.b * qkd - obj.m*obj.g * obj.l*sin(qk))/obj.I];
            df_dx = [0, 1; - obj.m*obj.g * obj.l*cos(qk)/obj.I, -obj.b/obj.I];
            df_du = [0; 1/obj.I];
            
        end
        
        % equality constraints, c(x, u) = 0, and first derivatives
        % current we have none, so we write a 1D trivial const (0 = 0)
        function [c, dc_dx, dc_du] = equality_const(obj, xk, uk)
            
            c = 0;
            dc_dx = zeros(1, obj.nx);
            dc_du = zeros(1, obj.nu);
            
        end
        
        % inequality constraints, c(x, u) <= 0, and first derivatives
        % currently constraints uk to be -t_m <= uk <= t_m
        function [c, dc_dx, dc_du] = inequality_const(obj, xk, uk)
            c = [uk; -uk] - [obj.t_m; obj.t_m];
            dc_dx = zeros(obj.niq, obj.nx);
            dc_du = [1; -1];
        end
        
        % measure the difference between two state vectors
        function dx = state_diff(obj, x1, x2)
            d = mod(x1(1) - x2(1) + pi, 2*pi) - pi;
            dx = [d; x1(2) - x2(2)];
        end
    end
end

