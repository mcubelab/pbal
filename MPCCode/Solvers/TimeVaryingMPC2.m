classdef TimeVaryingMPC2
    % MPC controller driving a non-linear system of the following form
    % xd_k = f(x_k, u_k)
    % ceq(x_k, u_k) = 0
    % ciq(x_k, u_k) => 0
    % to the trajectory Xnom, Unom
            
    
    properties
        
        Ntraj   % trajectory length
        Nmpc    % controller horizon
        
        x0    % nominal (goal) state 
        u0    % nominal (goal) input
        
        R   % one-step input cost 0.5 * du^T * R * du
        Q   % one-step state cost 0.5 * dx^T * Q * dx
        QN  % final state cost 0.5 * dxN^T * QN * dxN
        
        sys % non-linear system to control
        
        nx  % state dimension
        nu  % input dimension
        neq % number of equality constraints
        niq % number of ineqaulity constraints
        
        dt % control rate
        
        % store QP Matrices
        big_H
        big_Aiq
        big_Aeq
        big_biq
        big_beq
        
        % linearization matrices
        A_dyn
        B_dyn
        
        A_eq
        B_eq
        c_eq
        
        A_iq
        B_iq
        c_iq       
        
        
    end
    
    methods
        
        % initialize
        function obj = TimeVaryingMPC2(system, params)

            obj.Ntraj = params.Ntraj;   % trajectory length
            obj.Nmpc = params.Nmpc;     % controller horizon
            
            obj.x0 = params.x0;     % nominal state trajectory
            obj.u0 = params.u0;     % nominal input trajectory
            
            obj.sys = system;  % system to control
            
            % dimension of system
            obj.nx = obj.sys.nx;
            obj.nu = obj.sys.nu;
            obj.neq = obj.sys.neq;
            obj.niq = obj.sys.niq;
            
            % control rate
            obj.dt = params.dt; 
            
            % set cost matrices
            obj.QN = params.QN;
            obj.Q = params.Q;
            obj.R = params.R;
            
            
            % build cost matrix
            obj.big_H = blkdiag(kron(eye(obj.Nmpc - 1), obj.Q), obj.QN, ...
                kron(eye(obj.Nmpc), obj.R));
             
            % ineqaulity constraints
            obj.big_Aiq = zeros(obj.Nmpc * obj.niq, obj.Nmpc * (obj.nx + obj.nu));
            obj.big_biq = zeros(obj.Nmpc * obj.niq, 1);
            
            % eqaulity constraints
            obj.big_Aeq = zeros(obj.Nmpc * (obj.nx + obj.neq), obj.Nmpc * (obj.nx + obj.nu));
            obj.big_beq = zeros(obj.Nmpc * (obj.nx + obj.neq), 1); 
            
            % build linearixation
            obj.update_linearization();
            

        end
        
        % solve the MPC problem on the horizon Nmpc
        function [Xpredicted, Upredicted] = run_mpc(obj, xk, update_linearization_flag)
            % k is the current index (time)
            % xk is the state at that index
            
   
            % build linearization
            if update_linearization_flag
                obj = obj.update_linearization();            
            end
            
            % build matrices
            obj = obj.build_qp_matrices(xk, update_linearization_flag);
            
            % solve for z = [x_1, ..., x_n, u_0, ..., u_{n-1}]^T;
            [z, ~, exitflag] = quadprog(obj.big_H, [], obj.big_Aiq, obj.big_biq, ...
                obj.big_Aeq, obj.big_beq, [], [], [], optimoptions('quadprog', ...
                'maxiterations', 1e3, 'display', 'none'));
            
            if exitflag < 0
                error('QP solver failed')
            end
            
            Xpredicted = z(1:(obj.Nmpc)*obj.nx); % state sequence
            Upredicted = z((obj.Nmpc) * obj.nx + 1:end); % input sequence
            
        end
        
        function obj = build_qp_matrices(obj, xk, update_linearization_flag)
            
            if update_linearization_flag
                % transition
                bigAdyn = zeros(obj.nx * obj.Nmpc);
                for k = 1:obj.Nmpc
                    xk_ind = (k-1)*obj.nx+1: k*obj.nx;
                    if k == 1
                        bigAdyn(xk_ind, xk_ind) = eye(obj.nx);
                    else
                        bigAdyn(xk_ind, (k-2)*obj.nx+1:k*obj.nx) = ...
                            [-obj.A_dyn, eye(obj.nx)];
                    end
                end
                bigBdyn = -kron(eye(obj.Nmpc), obj.B_dyn);

                % equality constraint
                bigAeq = kron(eye(obj.Nmpc), obj.A_eq);
                bigBeq = kron(eye(obj.Nmpc), obj.B_eq);
                bigceq = repmat(obj.c_eq, obj.Nmpc, 1);

                % inequality constraint
                bigAiq = kron(eye(obj.Nmpc), obj.A_iq);
                bigBiq = kron(eye(obj.Nmpc), obj.B_iq);
                obj.big_biq = repmat(obj.c_iq, obj.Nmpc, 1);

                % collect inequalty constraints for QP
                obj.big_Aiq = [bigAiq, bigBiq];

                % collect equality constraints for QP
                obj.big_Aeq = [bigAdyn, bigBdyn;
                    bigAeq,  bigBeq];
            end
            
            % update starting constraint
            obj.big_beq = [obj.A_dyn * obj.sys.state_diff(xk, obj.x0); ...
                zeros(obj.nx * (obj.Nmpc-1), 1); ...
                bigceq]; 
            
        end

        % set and update constraint matrices
        function obj = update_linearization(obj)
            
            if isempty(obj.A_dyn) % first call - initialize matrices
                obj.A_dyn = zeros(obj.nx, obj.nx);
                obj.B_dyn = zeros(obj.nx, obj.nu);

                obj.A_eq = zeros(obj.neq, obj.nx);
                obj.B_eq = zeros(obj.neq, obj.nu);
                obj.c_eq = zeros(obj.neq, 1);

                obj.A_iq = zeros(obj.niq, obj.nx);
                obj.B_iq = zeros(obj.niq, obj.nu);
                obj.c_iq = zeros(obj.niq, 1);
            end

            % linearize transition constraint
            [~, Ak, Bk] = obj.sys.dynamics(obj.x0, obj.u0);
            obj.A_dyn = eye(obj.nx) + obj.dt * Ak;
            obj.B_dyn = obj.dt * Bk;

            % linearize equality const
            [ceq, obj.A_eq, obj.B_eq] = obj.sys.equality_const(obj.x0, obj.u0);
            obj.c_eq = -ceq;

            % linerize inequality const
            [ciq, obj.A_iq, obj.B_iq] = obj.sys.inequality_const(obj.x0, obj.u0);
            obj.c_iq = -ciq;

        end
        
    end
end


