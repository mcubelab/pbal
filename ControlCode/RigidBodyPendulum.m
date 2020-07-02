classdef RigidBodyPendulum
    % A rod of mass (m) and length(l) pivoting about its top end in the
    % gravity plane
    
    properties
        
        % user specified
        m;   % mass (kg)
        l;   % pendulum length (m)
        dt;  % time-step
        t_m  % control torque limit (N * m)
        b    % damping coeff
        
        % fixed/derived
        g;              % acceleration due to gravity (kg/m^2)
        I;              % inertia about COM (kg^2 * m^2)
        fmincon_opt;    % options for fmincon
        
        
        % dimensions
        nq;                 % config dimension
        nv;                 % gen. gevelocity dimension
        nu;                 % input dimension
        neq;                % # of equality const
        niq;                % # of inequality const
        
    end
    
    methods
        
        % initialize
        function obj = RigidBodyPendulum(params)
            
            obj.m = params.m;       % mass  (kg)
            obj.l = params.l;       % length (m)
            obj.dt = params.dt;     % time-step
            obj.t_m = params.t_m;   % control torque limit (N*m)
            obj.b = params.b;       % damping coefficient
            
            obj.g = 9.81;                   % gravity (m/s^2)
            obj.I = obj.l^2 * obj.m^2/12;   % inertia (kg^2 * m^2)
            
            % options for fmincon
            obj.fmincon_opt =  optimoptions('fmincon', 'algorithm', 'sqp', ...
                'display', 'final', 'specifyconstraintgradient', true, ...
                'steptolerance', 1e-8, 'constrainttolerance', 1.e-4, ...
                'maxfunctionevaluations', 2000);
            
            % dimensions
            obj.nq = 3;
            obj.nv = 3;
            obj.nu = 3;
            obj.neq = 4;
            obj.niq = 2;
        end
        
        % given xk, find xkp1 and uk (pivot forces; input torque) that
        % that satisfy the following equations:
        % (1) xkp1 = obj.forward_dyn_euler(xk, uk)
        % (2) obj.pivot_const(xk) = 0
        % (3) A * u <= b where [A, b] = obj.build_input_const()      
        
        function [xkp1, uk] =  dynamics_solve(obj, xk, uk)
            
            qk = xk(1:obj.nq);
            qkd = xk(obj.nq + (1:obj.nv));
            
            M = obj.build_mass_matrix();
            c = obj.build_coriolis_and_potential(qkd);
            B  = obj.build_input_matrix(qk);
            
            Adyn = [eye(6), [zeros(3); obj.dt * -(M\B)]];
            bdyn = xk + obj.dt * [qkd; -(M\c)];
            
            [Au, bu] = obj.input_const_fmincon();
            
            z = fmincon(@(z) 0, [xk; uk], Au, bu, Adyn, bdyn , ...
                [], [], @obj.pivot_const_fmincon, obj.fmincon_opt);
            
            xkp1 = z(1:6);
            uk = z(7:9);
        end
        
        % wrapper for pivot constraint so we can feed it into fmincon
        function [c, ceq, dc, dceq] = pivot_const_fmincon(obj, zk)
            
            xk = zk(1:(obj.nq + obj.nv));
            [ceq, dc_dxk, dc_duk] = obj.pivot_const(xk);
            dceq = [dc_dxk, dc_duk]';
            c = [];
            dc = [];
        end
    
        % wrapper for input constraint so we can feed it into fmincon
        function [A, b] = input_const_fmincon(obj)
            [Au, b] = obj.build_input_const();
            A = [zeros(obj.niq, obj.nq + obj.nv), Au];
            
        end
        
        % forward dynamics using euler integration 
        % [qkp1; qdkp1] = [qk; qkd] + dt*[qkd; qkdd]
        % qkdd = M^{-1} * (B(q) * uk - c(qd))
        % xk = [qk; qkd]
        function [xkp1, dxkp1_dx, dxkp1_du] = forward_dyn_euler(obj, xk, uk)
            
            % separate state
            qk = xk(1:3);
            qkd = xk(4:6);
            
            % build manipulator eqn
            M = obj.build_mass_matrix();
            c = obj.build_coriolis_and_potential(qkd);
            B = obj.build_input_matrix(qk);
            
            % solve for qkdd and integrate
            xkp1 = xk + obj.dt * [qkd; M\(B * uk - c)];
            
            % derivative w.r.t. xk
            dc_dqd = [0, 0, 0;
                0, 0, 0;
                0, 0, obj.b];
            
            dBu_dq = [0, 0, 0; 
                0, 0, 0
                0, 0, 0.5*obj.l*(sin(qk(3)) * uk(1) - cos(qk(3)) * uk(2))] ;
            
            dxkp1_dx = eye(obj.nq + obj.nv) + ...
                obj.dt * [zeros(obj.nq), eye(obj.nv); M\dBu_dq ,-M\dc_dqd];
            
            % derivative w.r.t to uk
            dxkp1_du = obj.dt * [zeros(obj.nq, obj.nu); (M\B)];
            
        end
        
        % B(q) in M(q) qdd + c(qd) = B(q) * u
        function B = build_input_matrix(obj, qk)
            
            tht = qk(3);
            
            B = [1, 0, 0;
                0, 1, 0;
                -0.5 * obj.l * cos(tht), -0.5 * obj.l * sin(tht), 1];            
        end
        
        % M(q) in M(q) qdd + c(qd) = B(q) * u
        function M = build_mass_matrix(obj)
            M = [obj.m, 0, 0;
                0, obj.m, 0;
                0, 0, obj.I];
        end
        
        % c(qd) in M*qdd + c(qd) = B(q) * u
        % currently just gravity and damping in theta
        function c = build_coriolis_and_potential(obj, qkd)
            c = [0; obj.m * obj.g; 0] + [0; 0; obj.b*qkd(3)];
        end
        
        % this function evaluates the pivot constraint -- the top end of the
        % rod is fixed to (0,0) -- on positions and velocities
        function [c, dc_dxk, dc_duk] = pivot_const(obj, xk)
            
            x = xk(1);          % x-pos of COM
            y = xk(2);          % y-pos of COM
            tht = xk(3);        % rotation of obj
            
            % velocity of COM
            qd = xk(obj.nq + (1:obj.nv));
            
            % position constraints: phi(q) = 0
            phi = [x - 0.5 * obj.l * sin(tht);
                y + 0.5 * obj.l * cos(tht)];
            
            % derivative of phi w.r.t to q
            dphi_dq = [1, 0, -0.5 * obj.l * cos(tht);
                0, 1, - 0.5 * obj.l * sin(tht)] ;
            
            % c = [phi(q); dphi_dq(q) * qd ] = 0
            c = [phi; dphi_dq * qd];
            
            %dc_dx = [dphi_dq, 0_{2x3}; d2phi_dq2*qd, dphi_dq]
            dc_dxk = [dphi_dq, zeros(obj.neq/2, obj.nv);
                [0, 0, 0.5 * obj.l * sin(tht) * qd(3);
                0, 0, -0.5 * obj.l * cos(tht) * qd(3)], dphi_dq];
            
            %dc_du = 0
            dc_duk = zeros(obj.neq, obj.nu);
            
        end
        
        % this function puts constraints on the input currently just bound
        % constraints on torque %-t_m <= tau <= t_m
        function [Au, bu] = build_input_const(obj)
            Au = [0, 0, -1; 0, 0, 1];
            bu = [obj.t_m; obj.t_m];
        end       
        
    end
end

