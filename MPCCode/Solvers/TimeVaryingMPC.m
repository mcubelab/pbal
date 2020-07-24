classdef TimeVaryingMPC
    % MPC controller driving a non-linear system of the following form
    % xd_k = f(x_k, u_k)
    % ceq(x_k, u_k) = 0
    % ciq(x_k, u_k) => 0
    % to the trajectory Xnom, Unom
            
    
    properties
        
        Ntraj   % trajectory length
        Nmpc    % controller horizon
        
        Xnom    % nominal state trajectory
        Unom    % nominal input trajectory
        
        R   % one-step input cost 0.5 * du^T * R * du
        Q   % one-step state cost 0.5 * dx^T * Q * dx
        QN  % final state cost 0.5 * dxN^T * QN * dxN
        
        sys % non-linear system to control
        
        nx  % state dimension
        nu  % input dimension
        neq % number of equality constraints
        niq % number of ineqaulity constraints
        
        dt % control rate
        
        
        % store linearization
        A_dyn_traj
        B_dyn_traj
        
        A_eq_traj
        B_eq_traj
        c_eq_traj
        
        A_iq_traj
        B_iq_traj
        c_iq_traj
        
    end
    
    methods
        
        % initialize
        function obj = TimeVaryingMPC(system, params)

            obj.Ntraj = params.Ntraj;   % trajectory length
            obj.Nmpc = params.Nmpc;     % controller horizon
            
            obj.Xnom = params.Xnom;     % nominal state trajectory
            obj.Unom = params.Unom;     % nominal input trajectory
            
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
            
            % build time-varying linearization about nominal
            obj = obj.build_linearization();
            
        end
        
        % solve the MPC problem on the horizon Nmpc
        function [Xpredicted, Upredicted] = run_mpc(obj, k, xk)
            % k is the current index (time)
            % xk is the state at that index
            
            % build matrices
            [H, Aiq, biq, Aeq, beq] = obj.build_qp_matrices(k, xk);
            
            % solve for z = [x_1, ..., x_n, u_0, ..., u_{n-1}]^T;
            [z, ~, exitflag] = quadprog(H, [], Aiq, biq, Aeq, beq, [], [], [], ...
                optimoptions('quadprog', 'maxiterations', 1e3, ...
                'display', 'none'));
            
            if exitflag < 0
                error('QP solver failed')
            end
            
            Xpredicted = z(1:(obj.Nmpc)*obj.nx); % state sequence
            Upredicted = z((obj.Nmpc) * obj.nx + 1:end); % input sequence
            
        end
        
        function [H, Aiq, biq, Aeq, beq] = build_qp_matrices(obj, t0, xk)
            
            % build cost matrix
            H = blkdiag(kron(eye(obj.Nmpc - 1), obj.Q), obj.QN, ...
                kron(eye(obj.Nmpc), obj.R));
            
            % grab relevant matrices from linearization based on time
            if t0 + obj.Nmpc - 1 > obj.Ntraj
                
                A_dyn = cat(3, obj.A_dyn_traj(:, :, t0:obj.Ntraj), ...
                    repmat(obj.A_dyn_traj(:, :, obj.Ntraj), ...
                    1, 1, t0 + obj.Nmpc - obj.Ntraj - 1));
                B_dyn = cat(3, obj.B_dyn_traj(:, :, t0:obj.Ntraj), ...
                    repmat(obj.B_dyn_traj(:, :, obj.Ntraj), ...
                    1, 1, t0 + obj.Nmpc - obj.Ntraj - 1));
                
                A_eq  = cat(3, obj.A_eq_traj(:, :, t0:obj.Ntraj), ...
                    repmat(obj.A_eq_traj(:, :, obj.Ntraj), ...
                    1, 1, t0 + obj.Nmpc - obj.Ntraj - 1));
                B_eq = cat(3, obj.B_eq_traj(:, :, t0:obj.Ntraj), ...
                    repmat(obj.B_eq_traj(:, :, obj.Ntraj), ...
                    1, 1, t0 + obj.Nmpc - obj.Ntraj - 1));
                c_eq  = [obj.c_eq_traj(:, t0:obj.Ntraj), ...
                    repmat(obj.c_eq_traj(:, obj.Ntraj), ...
                    1, t0 + obj.Nmpc - obj.Ntraj - 1)];
                
                A_iq  = cat(3, obj.A_iq_traj(:, :, t0:obj.Ntraj), ...
                    repmat(obj.A_iq_traj(:, :, obj.Ntraj), ...
                    1, 1, t0 + obj.Nmpc - obj.Ntraj - 1));
                B_iq  = cat(3, obj.B_iq_traj(:, :, t0:obj.Ntraj), ...
                    repmat(obj.B_iq_traj(:, :, obj.Ntraj), ...
                    1, 1, t0 + obj.Nmpc - obj.Ntraj - 1));
                c_iq  = [obj.c_iq_traj(:, t0:obj.Ntraj), ...
                    repmat(obj.c_iq_traj(:, obj.Ntraj), ...
                    1, t0 + obj.Nmpc - obj.Ntraj - 1)];
                
            else
                A_dyn = obj.A_dyn_traj(:, :, t0+(0:obj.Nmpc-1));
                B_dyn = obj.B_dyn_traj(:, :, t0+(0:obj.Nmpc-1));
                
                A_eq  = obj.A_eq_traj(:, :, t0+(0:obj.Nmpc-1));
                B_eq = obj.B_eq_traj(:, :, t0+(0:obj.Nmpc-1));
                c_eq  = obj.c_eq_traj(:, t0+(0:obj.Nmpc-1));
                
                A_iq  = obj.A_iq_traj(:, :, t0+(0:obj.Nmpc-1));
                B_iq  = obj.B_iq_traj(:, :, t0+(0:obj.Nmpc-1));
                c_iq  = obj.c_iq_traj(:, t0+(0:obj.Nmpc-1));
            end
            
            % transition
            bigAdyn = zeros(obj.nx * obj.Nmpc);
            for k = 1:obj.Nmpc
                xk_ind = (k-1)*obj.nx+1: k*obj.nx;
                if k == 1
                    bigAdyn(xk_ind, xk_ind) = eye(obj.nx);
                else
                    bigAdyn(xk_ind, (k-2)*obj.nx+1:k*obj.nx) = ...
                        [-A_dyn(:, :, k), eye(obj.nx)];
                end
            end
            bigBdyn = -obj.threeDToblkdiag(B_dyn);
            
            % equality constraint
            bigAeq = obj.threeDToblkdiag(A_eq);
            bigBeq = obj.threeDToblkdiag(B_eq);
            bigceq = c_eq(:);
            
            % inequality constraint
            bigAiq = obj.threeDToblkdiag(A_iq);
            bigBiq = obj.threeDToblkdiag(B_iq);
            bigciq = c_iq(:);
            
            % collect inequalty constraints for QP
            Aiq = [bigAiq, bigBiq];
            biq = bigciq;
            
            % collect equality constraints for QP
            Aeq = [bigAdyn, bigBdyn;
                bigAeq,  bigBeq];
            beq = [A_dyn(:, :, 1) * obj.sys.state_diff(xk, obj.Xnom(:,t0)); ...
                zeros(obj.nx * (obj.Nmpc-1), 1); ...
                bigceq];
            
        end
        
        % helper function converts 3D trajectory of matrices (time along
        % third dimension) to a block diagonal matrix 
        function blk_mat = threeDToblkdiag(obj, threeD)
            kcell = num2cell(threeD,[1,2]);
            blk_mat=blkdiag(kcell{:});
        end
        
        % set and update constraint matrices
        function obj = build_linearization(obj)
            
            obj.A_dyn_traj = zeros(obj.nx, obj.nx, obj.Ntraj);
            obj.B_dyn_traj = zeros(obj.nx, obj.nu, obj.Ntraj);
            
            obj.A_eq_traj = zeros(obj.neq, obj.nx, obj.Ntraj);
            obj.B_eq_traj = zeros(obj.neq, obj.nu, obj.Ntraj);
            obj.c_eq_traj = zeros(obj.neq, obj.Ntraj);
            
            obj.A_iq_traj = zeros(obj.niq, obj.nx, obj.Ntraj);
            obj.B_iq_traj = zeros(obj.niq, obj.nu, obj.Ntraj);
            obj.c_iq_traj = zeros(obj.niq, obj.Ntraj);
            
            for k = 1:obj.Ntraj
                
                % k-th state and input
                xk = obj.Xnom(:, k);
                uk = obj.Unom(:, k);
                
                % linearize transition constraint
                [~, Ak, Bk] = obj.sys.dynamics(xk, uk);
                obj.A_dyn_traj(:, :, k) = eye(obj.nx) + obj.dt * Ak;
                obj.B_dyn_traj(:, :, k) = obj.dt * Bk;
                
                % linearize equality const
                [ceq, Aeq, Beq] = obj.sys.equality_const(xk, uk);
                obj.A_eq_traj(:, :, k) = Aeq;
                obj.B_eq_traj(:, :, k) = Beq;
                obj.c_eq_traj(:, k) = -ceq;
                
                % linerize inequality const
                [ciq, Aiq, Biq] = obj.sys.inequality_const(xk, uk);
                obj.A_iq_traj(:, :, k) = Aiq;
                obj.B_iq_traj(:, :, k) = Biq;
                obj.c_iq_traj(:, k) = -ciq;
                
            end
            
        end
        
    end
end


