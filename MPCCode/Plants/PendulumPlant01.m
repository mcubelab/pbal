classdef PendulumPlant01
    % A rod of mass (m) and length(l) pivoting about its top end in the
    % gravity plane
    
    properties
        
        % user specified
        m;   % mass (kg)
        l;   % pendulum length (m)
        t_m  % control torque limit (N * m)
        b    % damping coeff
        mu   % coefficient of friction
        
        % fixed/derived
        g;              % acceleration due to gravity (kg/m^2)
        I;              % inertia about pivot (kg^2 * m^2)
        fmincon_opt;    % options for fmincon
        
        
        % dimensions
        nq;                 % config dimension
        nv;                 % gen.;; gevelocity dimension
        nx;
        nu;                 % input dimension
        neq;                % # of equality const
        niq;                % # of inequality const
        
        %Simulation Environment instance representing this system
        MyEnvironment;
        
        %Rigid body representing the pendulum in the simulation
        pendulum_rigid_body_object;
        
        %Generalized force representing the control input in the simulation
        ControlInput;
        
    end
    
    methods
        
        % initialize
        function obj = PendulumPlant01(params)
            
            obj.m = params.m;       % mass  (kg)
            obj.l = params.l;       % length (m)
            obj.t_m = params.t_m;   % control torque limit (N*m)
%             obj.b = params.b;       % damping coefficient
            obj.g = params.g;                   % gravity (m/s^2)
            obj.mu = params.mu;     % coefficient of friction
            obj.I = obj.l^2 * obj.m^2/3;   % inertia (kg^2 * m^2)
            
            
            plist = [0,0;0,-obj.l];
            r_cm  = [0;-obj.l/2];
            I_com = obj.m*(obj.l)^2/12;
            
            obj.pendulum_rigid_body_object=PolygonRigidBody(plist, r_cm, obj.m, I_com);
            
%             sticking_constraint_ground=PolygonConstraint();
%             sticking_constraint_ground.StickingContactOneBody(obj.pendulum_rigid_body_object,[0;0],[0;0]);
            
            myGravity=PolygonGeneralizedForce();
            myGravity.gravity(obj.pendulum_rigid_body_object,[0;-obj.g]);
            
            obj.ControlInput=PolygonGeneralizedForce();
            obj.ControlInput.external_wrench(obj.pendulum_rigid_body_object,[0;0]);
            
            obj.MyEnvironment=SimulationEnvironment();
            
            obj.MyEnvironment.addRigidBody(obj.pendulum_rigid_body_object);
%             obj.MyEnvironment.addConstraint(sticking_constraint_ground);
            obj.MyEnvironment.addGeneralizedForce(myGravity);
            obj.MyEnvironment.addGeneralizedForce(obj.ControlInput);

            
            % dimensions
            obj.nq = 3;
            obj.nv = 3;
            obj.nx = obj.nq + obj.nv;
            obj.nu = 3;
            obj.neq = 2;
            obj.niq = 4;
            
            
        end

        
        
        % continuous forward dynamics
        % [qkd, qkdd] = [qkd; M^{-1} * (B(q) * uk - c(qd))]
        % xk = [qk; qkd]
        % qk = [x_pivot; y_pivot; theta=angle w/respect positive x axis]
        % qkd = d/dt (qk)
        % uk = [lambda_x; lambda_y; tau]
        % fn and ft are forces exerted at the robot contact
        % fn and ft are defined in the contact frame
        % tau is a moment exerted on the object by the robot
        % lambda_x is force exerted in the x direction of the world frame
        % lambda_y is force exerted in the y direction of the world frame
        % lambda_x/lambda_y are exerted at the pivot of the object
        % f = d/dt xk
        % df_dx partial f / partial xk
        % df_du partial f / partial uk
        function [f, df_dx, df_du] = dynamics(obj, xk, uk)
            obj.MyEnvironment.assign_coordinate_vector(xk(1:3));
            obj.MyEnvironment.assign_velocity_vector(xk(4:6));
            obj.ControlInput.set_wrench_value(uk);
            
            obj.MyEnvironment.computeAccelerations();
            
            f= obj.MyEnvironment.build_acceleration_vector();

            delta_val=.0001;
            for count=1:6
                delta_xk=zeros(6,1);
                delta_xk(count)=delta_val;
                
                xk_plus_temp=xk+delta_xk;
                xk_minus_temp=xk-delta_xk;
                
                obj.MyEnvironment.assign_coordinate_vector(xk_plus_temp(1:3));
                obj.MyEnvironment.assign_velocity_vector(xk_plus_temp(4:6));
                obj.ControlInput.set_wrench_value(uk);

                obj.MyEnvironment.computeAccelerations();

                f_plus= obj.MyEnvironment.build_acceleration_vector();
                
                obj.MyEnvironment.assign_coordinate_vector(xk_minus_temp(1:3));
                obj.MyEnvironment.assign_velocity_vector(xk_minus_temp(4:6));
                obj.ControlInput.set_wrench_value(uk);

                obj.MyEnvironment.computeAccelerations();

                f_minus= obj.MyEnvironment.build_acceleration_vector();
                
                df_dx(:,count)=(f_plus-f_minus)/(2*delta_val);
            end

            
            for count=1:3
                delta_uk=zeros(3,1);
                delta_uk(count)=delta_val;
                
                uk_plus_temp=uk+delta_uk;
                uk_minus_temp=uk-delta_uk;
                
                obj.MyEnvironment.assign_coordinate_vector(xk(1:3));
                obj.MyEnvironment.assign_velocity_vector(xk(4:6));
                obj.ControlInput.set_wrench_value(uk_plus_temp);

                obj.MyEnvironment.computeAccelerations();

                f_plus= obj.MyEnvironment.build_acceleration_vector();
                
                obj.MyEnvironment.assign_coordinate_vector(xk(1:3));
                obj.MyEnvironment.assign_velocity_vector(xk(4:6));
                obj.ControlInput.set_wrench_value(uk_minus_temp);

                obj.MyEnvironment.computeAccelerations();

                f_minus= obj.MyEnvironment.build_acceleration_vector();
                
                df_du(:,count)=(f_plus-f_minus)/(2*delta_val);
            end
            
                     
%             % separate state
%             qk = xk(1:obj.nq);
%             qkd = xk(obj.nq + (1:obj.nv));
%             
%             % build manipulator eqn
%             %             [M, dM_dq] = obj.build_mass_matrix(qk);
%             [c, dc_dx] = obj.build_coriolis_and_potential(qk, qkd);
%             B = obj.build_input_matrix();
%             %
%             %             % compute derivative of M inverse
%             %             Minv = M\eye(obj.nq);
%             %             dMinv_dq = test(q);
%             [Minv, dMinv_dq] = build_inverse_mass_matrix(obj, qk);
%             
%             % solve for qkdd and integrate
%             f = [qkd; Minv * (B * uk - c)];
%             
%             % deriative w.r.t to xk
%             df_dx =  [zeros(obj.nq), eye(obj.nv);
%                 -Minv * dc_dx(:, 1:obj.nq) + kron((B*uk - c)', eye(obj.nq))*dMinv_dq, ...
%                 -Minv * dc_dx(:, obj.nq+(1:obj.nv))];
%             
%             % derivative w.r.t to uk
%             df_du = [zeros(obj.nq, obj.nu); (Minv * B)];
            
        end
        
        % JUST A VELOCITY CONSTRAINT
        % equality constraints, c(x, u) = 0, and first derivatives
        % current this is the constraint the pin-joint has on the
        % velocity 
        % xk = [qk; qkd]
        % qk = [x_pivot; y_pivot; theta=angle w/respect positive x axis]
        % qkd = d/dt (qk)
        % uk = [fn, ft, lambda_x, lambda_y, tau]
        % sticking constraint of the pivot on the velocity
        % c is just the velocity error function [vx_error,vy_error]
        % dc_dx is partial c / partial xk
        % dc_du is partial c / partial uk
        function [c, dc_dx, dc_du] = equality_const(obj, xk, uk)
            
            obj.MyEnvironment.assign_coordinate_vector(xk(1:3));
            obj.MyEnvironment.assign_velocity_vector(xk(4:6));
            obj.ControlInput.set_wrench_value(uk);
            
            c=obj.pendulum_rigid_body_object. rigid_body_velocity([0;0]);
            [~,~,Dx,Dy,~,~]=obj.pendulum_rigid_body_object.rigid_body_position_derivatives([0;0]);
            
%             [~,~,Dx,Dy,~,~]=obj.rigidBody1.rigid_body_position_derivatives(obj.pin1);
%             F(obj.rigidBody1.coord_index)=obj.external_wrench_val(1)*Dx'+obj.external_wrench_val(2)*Dy'+obj.external_wrench_val(3)*[0;0;1];
            
            % velocity of pivot
%             qd = xk(obj.nq + (1:obj.nv));               
            
            % pivot const on velocity
%             dphi_dq = [1, 0, 0; 0, 1, 0];
%             c = dphi_dq * qd;
            
%             dx_dx=[Dx;Dy];
%             dc_dx = [zeros(obj.neq, obj.nq), dphi_dq];
            
            dc_du = zeros(2, length(uk));
            
        end
        
        % inequality constraints, c(x, u) <= 0, and first derivatives
        % figure this one out later
        function [c, dc_dx, dc_du] = inequality_const(obj, xk, uk)
            
            fx = uk(1);
            fy = uk(2); 
            tau = uk(3); 
            
            c = [tau; -tau; fx - obj.mu*fy; -fx - obj.mu*fy] ...
                - [obj.t_m; obj.t_m; 0; 0];
            dc_dx = zeros(obj.niq, obj.nx);
            dc_du = [0, 0, 1; 0, 0, -1; 1 -obj.mu, 0; -1 -obj.mu, 0];
        end
        
        % measure the difference between two state vectors
        function dx = state_diff(obj, x1, x2)
            d = mod(x1(1) - x2(1) + pi, 2*pi) - pi;
            dx = [d; x1(2:end) - x2(2:end)];
        end
        
        %%%%%%%% Helper Functions %%%%%%%%%%%
        
        % B(q) in M(q)*qdd + c(q, qd) = B * u
        function B = build_input_matrix(obj)
            
            B = [1, 0, 0;
                0, 1, 0;
                0, 0, 1];
        end
        


   
        
    end
end

