classdef ConstrainedLinearSystem
    % Generate a CT linear system of the following form: 
    % xd = A * x + B * u
    % Aeq * x + Beq * u = ceq
    % Aiq * x + Biq * u <= ciq
    properties
                
        % user specified transition dyanmics
        A;
        B;
        
        % user specified equality const
        Aeq;
        Beq;
        ceq;
        
        % user specified inequality const
        Aiq;
        Biq;
        ciq;
        
        nx;     % state dimension
        nu;     % input dimension
        neq;    % # of equality const
        niq;    % # of inequality const
        
    end
    
    methods
        
        % initialize
        function obj = ConstrainedLinearSystem(params)
                        
            obj.nx = size(params.A, 1);
            obj.nu = size(params.B, 2);
            obj.neq = size(params.Aeq, 1);
            obj.niq = size(params.Aiq, 1);
            
            obj.A = params.A;
            obj.B = params.B;
            
            obj.Aeq = params.Aeq;
            obj.Beq = params.Beq;
            obj.ceq = params.ceq;
            
            obj.Aiq = params.Aiq;
            obj.Biq = params.Biq;
            obj.ciq = params.ciq;          
            
        end
        
        % dynamics function (xd = A*x + B*u) and first derivatives
        function [f, df_dx, df_du] = dynamics(obj, xk, uk)
            f = obj.A * xk + obj.B * uk;
            df_dx = obj.A;
            df_du = obj.B;
        end
        
        % equality constraints (Aeq * x + Beq * u = ceq) and first derivatives
        function [c, dc_dx, dc_du] = equality_const(obj, xk, uk)
            c = obj.Aeq * xk + obj.Beq * uk - obj.ceq;
            dc_dx = obj.Aeq;
            dc_du = obj.Beq;
        end
        
        % ineqaulity constraints (Aiq * x + Biq * u <= ciq) and first derivatives
        function [c, dc_dx, dc_du] = inequality_const(obj, xk, uk)
            c = obj.Aiq * xk + obj.Biq * uk - obj.ciq;
            dc_dx = obj.Aiq;
            dc_du = obj.Biq;
        end
        
        % measure the difference between two state vectors
        function dx = state_diff(obj, x1, x2)
            dx = x1 - x2;
        end
    end
end

