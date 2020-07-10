classdef TimeVaryingMPC
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        Ntraj   % trajectory length
        Nmpc    % controller horizon
        
        Xnom    % nominal state trajectory
        Unom    % nominal input trajectory
        
        R   % one-step input cost 0.5 * du^T * R * du
        Q   % one-step state cost 0.5 * dx^T * Q * dx
        QN  % final state cost 0.5 * dxN^T * QN * dxN
        
        NLS % non-linear system to control
        
        nx  % state dimension
        nu  % input dimension
        neq % number of equality constraints
        niq % number of ineqaulity constraints
        
        
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
        function obj = TimeVaryingMPC(NonLinearSystem, params)
            % MPC controller for a non-linear system of the following form
            % x_{k+1} = f(x_k, u_k)
            % g(x_k, u_k) >=0
            % h(x_k, u_k) = 0
            
            obj.Ntraj = params.Ntraj;   % trajectory length
            obj.Nmpc = params.Nmpc;     % controller horizon
            
            obj.Xnom = params.Xnom;     % nominal state trajectory
            obj.Unom = params.Unom;     % nominal input trajectory
            
            obj.NLS = NonLinearSystem;  % non-linear system to control
            
            % dimension of LS
            obj.nx = obj.NLS.nq + obj.NLS.nv;
            obj.nu = obj.NLS.nu;
            obj.neq = obj.NLS.neq/2;
            obj.niq = obj.NLS.niq;
            
            % set cost matrices
            obj.QN = params.QN;
            obj.Q = params.Q;
            obj.R = params.R;
            
            % build time-varying linearization about nominal
            obj = obj.build_linearization();
            
        end
        
        
        function [Xpredicted, Upredicted] = run_mpc(obj, k, xk)
            % k is the current index (time)
            % xk is the state at that index
            
            [H, Aiq, biq, Aeq, beq] = obj.build_qp_matrices(k, xk);
            
            % solve for z = [x_1, ..., x_n, u_0, ..., u_{n-1}]^T;
            %             z0 = [bigX(obj.nx + (1:obj.n * obj.nx)); bigU];
            z = quadprog(H, [], Aiq, biq, Aeq, beq, [], [], [], ...
                optimoptions('quadprog', 'maxiterations', 1e3, ...
                'display', 'final'));
            
            Xpredicted = z(1:(obj.Nmpc)*obj.nx); % state sequence
            Upredicted = z((obj.Nmpc) * obj.nx + 1:end); % input sequence
            
        end
        
        function [H, Aiq, biq, Aeq, beq] = build_qp_matrices(obj, k, xk)
            
            H = blkdiag(kron(eye(obj.Nmpc - 1), obj.Q), obj.QN, ...
                kron(eye(obj.Nmpc), obj.R));
            
            
            % grab relevant matrices from linearization based on time
            if k + obj.Nmpc - 1 > obj.Ntraj
                
                A_dyn = cat(3, obj.A_dyn_traj(:, :, k:obj.Ntraj), ...
                    repmat(obj.A_dyn_traj(:, :, obj.Ntraj), ...
                    1, 1, k + obj.Nmpc - obj.Ntraj - 1));
                B_dyn = cat(3, obj.B_dyn_traj(:, :, k:obj.Ntraj), ...
                    repmat(obj.B_dyn_traj(:, :, obj.Ntraj), ...
                    1, 1, k + obj.Nmpc - obj.Ntraj - 1));
                
                A_eq  = cat(3, obj.A_eq_traj(:, :, k:obj.Ntraj), ...
                    repmat(obj.A_eq_traj(:, :, obj.Ntraj), ...
                    1, 1, k + obj.Nmpc - obj.Ntraj - 1));
                B_eq = cat(3, obj.B_eq_traj(:, :, k:obj.Ntraj), ...
                    repmat(obj.B_eq_traj(:, :, obj.Ntraj), ...
                    1, 1, k + obj.Nmpc - obj.Ntraj - 1));
                c_eq  = [obj.c_eq_traj(:, k:obj.Ntraj), ...
                    repmat(obj.c_eq_traj(:, obj.Ntraj), ...
                    1, k + obj.Nmpc - obj.Ntraj - 1)];
                
                A_iq  = cat(3, obj.A_iq_traj(:, :, k:obj.Ntraj), ...
                    repmat(obj.A_iq_traj(:, :, obj.Ntraj), ...
                    1, 1, k + obj.Nmpc - obj.Ntraj - 1));
                B_iq  = cat(3, obj.B_iq_traj(:, :, k:obj.Ntraj), ...
                    repmat(obj.B_iq_traj(:, :, obj.Ntraj), ...
                    1, 1, k + obj.Nmpc - obj.Ntraj - 1));
                c_iq  = [obj.c_iq_traj(:, k:obj.Ntraj), ...
                    repmat(obj.c_iq_traj(:, obj.Ntraj), ...
                    1, k + obj.Nmpc - obj.Ntraj - 1)];
                
            else
                A_dyn = obj.A_dyn_traj(:, :, k+(0:obj.Nmpc-1));
                B_dyn = obj.B_dyn_traj(:, :, k+(0:obj.Nmpc-1));
                
                A_eq  = obj.A_eq_traj(:, :, k+(0:obj.Nmpc-1));
                B_eq = obj.B_eq_traj(:, :, k+(0:obj.Nmpc-1));
                c_eq  = obj.c_eq_traj(:, k+(0:obj.Nmpc-1));
                
                A_iq  = obj.A_iq_traj(:, :, k+(0:obj.Nmpc-1));
                B_iq  = obj.B_iq_traj(:, :, k+(0:obj.Nmpc-1));
                c_iq  = obj.c_iq_traj(:, k+(0:obj.Nmpc-1));
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
            beq = [A_dyn(:, :, 1) * (xk - obj.Xnom(:, k)); ...
                zeros(obj.nx * (obj.Nmpc-1), 1); ...
                bigceq];
            
        end
        
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
            obj.c_eq_traj = zeros(obj.niq, obj.Ntraj);
            
            for k = 1:obj.Ntraj
                
                % k-th state and input
                xk = obj.Xnom(:, k);
                uk = obj.Unom(:, k);
                
                % linearize transition constraint
                [~, Ak, Bk] = obj.NLS.dynamics(xk, uk);
                obj.A_dyn_traj(:, :, k) = eye(obj.nx) + obj.NLS.dt*Ak;
                obj.B_dyn_traj(:, :, k) = obj.NLS.dt*Bk;

                % linearize equality const
                [kk, Ek, Fk] = obj.NLS.pivot_const(xk);
                obj.A_eq_traj(:, :, k) = Ek(obj.neq+(1:obj.neq),:);
                obj.B_eq_traj(:, :, k) = Fk(obj.neq+(1:obj.neq),:);
                obj.c_eq_traj(:, k) = -kk(obj.neq + (1:obj.neq));
%                 obj.A_eq_traj(:, :, k) = Ek(1:obj.neq,:);
%                 obj.B_eq_traj(:, :, k) = Fk(1:obj.neq,:);
%                 obj.c_eq_traj(:, k) = -kk(1:obj.neq);
                
                % linerize inequality const
                [Jk, lk] = obj.NLS.build_input_const();
                obj.A_iq_traj(:, :, k) = zeros(obj.niq, obj.nx);
                obj.B_iq_traj(:, :, k) = Jk;
                obj.c_iq_traj(:, k) = lk - Jk * uk;
                
            end
            
        end
        
    end
end


