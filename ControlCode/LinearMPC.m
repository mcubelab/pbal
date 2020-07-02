classdef LinearMPC
    % LinearMPCControl Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        R   % one-step input cost 0.5 * du^T * R * du
        Q   % one-step state cost 0.5 * dx^T * Q * dx
        n   % control horizon in steps
        
        % Stores A, B, E, F, k, G, H, l for LTI model of the following form
        % xkp1 = A * xk + B * u
        % E * xk + F * uk = k
        % G * uk + H * uk <= l
        
        LS
        nx  % state dimension
        nu  % input dimension
        neq % number of equality constraints
        niq % number of ineqaulity constraints
        
        % block matrices for solving direct transcription for LS via a QP
        
        % objective
        bigQ        % blkdiag(Q_1, Q_2, ..., Q_N)
        bigR        % blkdiag(R_1, R_2, ..., R_N)
        
        bigA % [A^1, ..., A_N]     %
        bigB % [B
        
        bigE % blkdiag(E_1, E_2, ..., E_N)
        bigF % blkdiag(F_1, F_2, ..., F_N)
        bigk % [k_1; k_2, ..., k_N]
        
        bigG % blkdiag(g_1, G_2, ..., G_N)
        bigJ % blkdiag(H_1, l_2, ..., H_N)
        bigl % [l_1; l_2, ..., l_N]     
        
    end
    
    methods
        function obj = LinearMPC(LinearSystem, n)
            %MPC controller for a linear system of the form
            % x_{k+1} = A * xk + B * uk
            % s.t. E * xk + F * uk = k
            % s.t. G * xk + J * uk <= l
            
            obj.n = n;   % control horizon
            
            obj.LS = LinearSystem; % struct for linear system to control
            
            % dimension of LS
            obj.nx = size(obj.LS.A, 1);
            obj.nu = size(obj.LS.B, 2);
            obj.neq = size(obj.LS.E, 1);
            obj.niq = size(obj.LS.G, 1);
            
            % pre-allocate matrices for solving direct transcription
            obj.bigQ = zeros(obj.n * obj.nx);
            obj.bigR = zeros(obj.n * obj.nu);
            
            obj.bigA = zeros(obj.n * obj.nx, obj.nx);
            obj.bigB = zeros(obj.n * obj.nx, obj.n * obj.nu);
            
            obj.bigE = zeros(obj.n * obj.neq, obj.n * obj.nx);
            obj.bigF = zeros(obj.n * obj.neq, obj.n * obj.nu);
            obj.bigk = zeros(obj.n * obj.neq, 1);
            
            obj.bigG = zeros(obj.n * obj.niq, obj.n * obj.nx);
            obj.bigJ = zeros(obj.n * obj.niq, obj.n * obj.nu);
            obj.bigl = zeros(obj.n * obj.niq, 1);
        end
        
        
        % solve QP
        function [bigX, bigU] = solve_qp_subproblem(obj, xi)
            
            % build matrices for quadprog
            
            % cost
            H = blkdiag(obj.bigQ, obj.bigR);

            % inequalty constraints
            Aiq = [obj.bigG, obj.bigJ];
            biq = obj.bigl;
            
            % equality constraints
            Aeq = [eye(obj.n * obj.nx), -obj.bigB;
                obj.bigE, obj.bigF];
            beq = [obj.bigA * xi; obj.bigk];
            
            % solve for z = [x_1, ..., x_n, u_0, ..., u_{n-1}]^T;
            z = quadprog(H, [], Aiq, biq, Aeq, beq, [], [], [], ...
                optimoptions('quadprog', 'maxiterations', 1e3, ...
                'display', 'final'));
            
            bigX = z(1:obj.n*obj.nx); % state sequence
            bigU = z(obj.n * obj.nx + 1:end); % input sequence
            
        end
        
        % set and update cost matrices
        function obj = set_cost_matrix(obj, Q, R)
            obj.Q = Q;
            obj.R = R;
            obj.bigQ = kron(eye(obj.n), obj.Q);
            obj.bigR = kron(eye(obj.n), obj.R);
        end
        
        % set and update constraint matrices
        function obj = update_constraint_mat(obj)
            
            for i = 1:obj.n                
                % input matrix
                for j = 1:i
                    obj.bigB((i-1)*obj.nx+1: i*obj.nx, (j-1)*obj.nu+1: j*obj.nu) = ...
                        obj.LS.A^(i-j) * obj.LS.B;
                end
                
                % state transition
                obj.bigA((i-1)*obj.nx+1: i*obj.nx, :) = obj.LS.A^i;              
            end
            
            % equality constraint
            obj.bigE = kron(eye(obj.n), obj.LS.E);
            obj.bigF = kron(eye(obj.n), obj.LS.F);
            obj.bigk = repmat(obj.LS.k, obj.n, 1);
            
            % equality constraint
            obj.bigG = kron(eye(obj.n), obj.LS.G);
            obj.bigJ = kron(eye(obj.n), obj.LS.J);
            obj.bigl = repmat(obj.LS.l, obj.n, 1);

        end
        
    end
end

