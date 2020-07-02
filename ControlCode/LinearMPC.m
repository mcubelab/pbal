classdef LinearMPC
    % LinearMPCControl Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        R   % one-step input cost 0.5 * (u - u0)^T * R * (u - u0)
        Q   % one-step state cost 0.5 * (x - x0)^T * Q * (x - x0)
        n   % control horizon in steps
        
        x0  % state we want to regulate
        u0  % nominal control effort
        
        nx  % state dimension
        nu  % input dimension
        neq % number of equality constraints
        niq % number of ineqaulity constraints
        
        A
        B
        E
        F
        k
        G
        J
        l
        
        bigA
        bigB
        
        bigE
        bigF
        bigk
        
        bigG
        bigJ
        bigl
        
        bigQ
        bigq
        bigR
        bigr
        
    end
    
    methods
        function obj = LinearMPC(LinearSystem, n)
            %MPC controller for a linear system of the form
            % x_{k+1} = A * x_k + B * u_k
            % s.t. E * x_k + F * u_k = k
            % s.t. G * x_k + J * u_k <= l
            
            obj.A = LinearSystem.A;
            obj.B = LinearSystem.B;
            
            obj.E = LinearSystem.E;
            obj.F = LinearSystem.F;
            obj.k = LinearSystem.k;

            obj.G = LinearSystem.G;
            obj.J = LinearSystem.J;
            obj.l = LinearSystem.l; 
            
            obj.n = n;
            
            obj.nx = size(obj.A, 1);
            obj.nu = size(obj.B, 2);
            obj.neq = size(obj.E, 1);
            obj.niq = size(obj.G, 1);
            
            obj.bigQ = zeros(obj.n * obj.nx);
            obj.bigq = zeros(obj.n * obj.nx, 1);
            obj.bigR = zeros(obj.n * obj.nu);
            obj.bigr = zeros(obj.n * obj.nu, 1);
            
            obj.bigA = zeros(obj.n * obj.nx, obj.nx);
            obj.bigB = zeros(obj.n * obj.nx, obj.n * obj.nu);
            
            obj.bigE = zeros(obj.n * obj.neq, obj.n * obj.nx);
            obj.bigF = zeros(obj.n * obj.neq, obj.n * obj.nu);
            obj.bigk = zeros(obj.n * obj.neq, 1); 
            
            obj.bigG = zeros(obj.n * obj.niq, obj.n * obj.nx);
            obj.bigJ = zeros(obj.n * obj.niq, obj.n * obj.nu);
            obj.bigl = zeros(obj.n * obj.niq, 1); 
        end
        
        function [bigX, bigU, dt] = solve_qp_subproblem(obj, xi)
            
            % build matrices for quadprog
            H = blkdiag(obj.bigQ, obj.bigR);
            f = [obj.bigq; obj.bigr];
            
            Aiq = [obj.bigG, obj.bigJ];
            biq = obj.bigl;
            
            Aeq = [eye(obj.n * obj.nx), -obj.bigB;
                obj.bigE, obj.bigF];
            beq = [obj.bigA * xi; obj.bigk];
            
            
            % z = [x_1, ..., x_n, u_0, ..., u_{n-1}]^T;
            tic;
            z = quadprog(H, f, Aiq, biq, Aeq, beq, [], [], [], ...
                optimoptions('quadprog', 'maxiterations', 1e3, ...
                'display', 'final'));
            dt = toc; 
            bigX = z(1:obj.n*obj.nx);
            bigU = z(obj.n * obj.nx + 1:end);
            
        end
        
        function obj = set_nominal_state(obj, x0)
            obj.x0 = x0;
        end
        
        function obj =  set_nominal_control(obj, u0)
            obj.u0 = u0;
        end
        
        function obj = set_cost_matrix(obj, Q, R)
            obj.Q = Q;
            obj.R = R;
        end
        
        function obj = update_cost_mat(obj)
            
            for i = 1:obj.n
                obj.bigQ((i-1)*obj.nx + 1: i*obj.nx, (i-1)*obj.nx +1: i*obj.nx) = ...
                    obj.Q;
                obj.bigR((i-1)*obj.nu+1: i*obj.nu, (i-1)*obj.nu+1: i*obj.nu) = ...
                    obj.R;
                obj.bigq((i-1)*obj.nx+1: i*obj.nx) = obj.Q * obj.x0;
                obj.bigr((i-1)*obj.nu+1: i*obj.nu) = obj.R * obj.u0;
            end
        end
        
        function obj = update_constraint_mat(obj)
            
            for i = 1:obj.n
                
                % input matrix
                for j = 1:i
                    obj.bigB((i-1)*obj.nx+1: i*obj.nx, (j-1)*obj.nu+1: j*obj.nu) = ...
                        obj.A^(i-j) * obj.B;
                end
                
                % state transition
                obj.bigA((i-1)*obj.nx+1: i*obj.nx, :) = obj.A^i;
                
                % equality constraint
                obj.bigE((i-1)*obj.neq+1: i*obj.neq, (i-1)*obj.nx+1: i*obj.nx) = ...
                    obj.E;
                obj.bigF((i-1)*obj.neq+1: i*obj.neq, (i-1)*obj.nu+1: i*obj.nu) = ...
                    obj.F;
                obj.bigk((i-1)*obj.neq+1: i*obj.neq) = obj.k; 
                
                % inequality constraint
                obj.bigG((i-1)*obj.niq+1: i*obj.niq, (i-1)*obj.nx+1: i*obj.nx) = ...
                    obj.G;
                obj.bigJ((i-1)*obj.niq+1: i*obj.niq, (i-1)*obj.nu+1: i*obj.nu) = ...
                    obj.J;
                obj.bigl((i-1)*obj.niq+1: i*obj.niq) = obj.l; 

            end
        end
        
    end
end

