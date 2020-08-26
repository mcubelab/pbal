classdef PendulumPlant01
    % A rod of mass (m) and length(l) pivoting about its top end in the
    % gravity plane
    
    properties
        
        % user specified
        m;   % mass (kg)
        l;   % pendulum length (m)
        t_m  % control torque limit (N * m)
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
        %In this case, there is no pivot constraint
        MyEnvironment;
        
        %Simulation Environment instance representing this system
        %In this case, the system is constrained
        MyEnvironment_constrained;
        
        %Rigid body representing the pendulum in the simulation
        pendulum_rigid_body_object;
        
        %Generalized force representing the control input in the simulation
        ControlInput;
        
        %Constraint associated with ground contact
        sticking_constraint_ground;
        
        %gravity
        myGravity;
        
    end
    
    methods
        
        % initialize
        function obj = PendulumPlant01(params)
            
            obj.m = params.m;       % mass  (kg)
            obj.l = params.l;       % length (m)
            obj.t_m = params.t_m;   % control torque limit (N*m)
            obj.g = params.g;                   % gravity (m/s^2)
            obj.mu = params.mu;     % coefficient of friction
            obj.I = obj.l^2 * obj.m^2/3;   % inertia (kg^2 * m^2)
            
            
            plist = [0,0;0,-obj.l];
            r_cm  = [0;-obj.l/2];
            I_com = obj.m*(obj.l)^2/12;
            
            obj.pendulum_rigid_body_object=PolygonRigidBody(plist, r_cm, obj.m, I_com);
            
            obj.sticking_constraint_ground=PolygonConstraint();
            obj.sticking_constraint_ground.StickingContactOneBody(obj.pendulum_rigid_body_object,[0;0],[0;0]);
            
            obj.myGravity=PolygonGeneralizedForce();
            obj.myGravity.gravity(obj.pendulum_rigid_body_object,[0;-obj.g]);
            
            obj.ControlInput=PolygonGeneralizedForce();
            obj.ControlInput.external_wrench(obj.pendulum_rigid_body_object,[0;0]);
            
            obj.MyEnvironment=SimulationEnvironment();
            
            obj.MyEnvironment.addRigidBody(obj.pendulum_rigid_body_object);
%             obj.MyEnvironment.addConstraint(sticking_constraint_ground);
            obj.MyEnvironment.addGeneralizedForce(obj.myGravity);
            obj.MyEnvironment.addGeneralizedForce(obj.ControlInput);
            
            obj.MyEnvironment_constrained=SimulationEnvironment();

            obj.MyEnvironment_constrained.addRigidBody(obj.pendulum_rigid_body_object);
            obj.MyEnvironment_constrained.addConstraint(obj.sticking_constraint_ground);
            obj.MyEnvironment_constrained.addGeneralizedForce(obj.myGravity);
            obj.MyEnvironment_constrained.addGeneralizedForce(obj.ControlInput);
            
            % dimensions
            obj.nq = 3;
            obj.nv = 3;
            obj.nx = obj.nq + obj.nv;
            obj.nu = 3;
            obj.neq = 2;
            obj.niq = 4;
            
        end
        
        
        function dXdt = my_KalmannPlantNoPartials(obj,X_in,u)
            %Unpack the system state/system parameters
            theta=X_in(1);  %angle of rigid body with resepect to -y axis
            %Specifically, angle that line segment connecting pivot to
            %robot contact with respect to the -y axis
            
            dtheta_dt=X_in(2); %time derivative of theta
            
            a=X_in(3); %coefficient representing gravity and moment of inertia
            %for an arbitrary rigid body, a=mgl/I where l is the distance
            %of the center of mass from the pivot, and I is the moment of
            %inertia of the body with respect to the pivot
            %for a simple pendulum, a=3/2 g/l
            
            b=X_in(4); %coefficient representing the moment of inertia
            %about the pivot locaction.
            %for an arbitrary rigid body, b=1/I
            %for the simple pendulum systme, b=3/ml^2
            
            theta_0=X_in(5); %offset angle from ray1 to ray 2
            %where ray1 is the ray from pivot to center of mass
            %and ray2 is the ray from pivot to contact point
            
            x_c=X_in(6); %x coordinate of the pivot location in world frame
            y_c=X_in(7); %y coordinate of the pivot location in world frame
            R=X_in(8); %distance from pivot to contact point

            
            %a=(3/2) g/l
            %b=3/(ml^2)
            
            params.l=1;
            params.g=(2/3)*a;
            params.m=3/b;
            params.mu=obj.mu;
            params.t_m=obj.t_m;
            
            obj.UpdateParams(params);
            
            theta_in=theta-theta_0;
            
            
            xk=[x_c;y_c;theta_in;0;0;dtheta_dt];
            uk=[0;0;u];
            f = obj.constrained_dynamics_no_partials(xk, uk);
            
            dXdt=zeros(8,1);
            dXdt(1)=f(3);
            dXdt(2)=f(6);
        end
        
        function [dXdt,J] = my_KalmannPlantWithPartials(obj,X_in,u)
            dXdt=obj.my_KalmannPlantNoPartials(X_in,u);
            
            delta_val=10^-6;
            J=zeros(length(dXdt),length(X_in));

            for count=1:length(X_in)
                delta_X_in=zeros(length(X_in),1);
                delta_X_in(count)=delta_val;
                
                X_in_plus_temp=X_in+delta_X_in;
                X_in_minus_temp=X_in-delta_X_in;
                
                f_plus=obj.my_KalmannPlantNoPartials(X_in_plus_temp,u);
                f_minus=obj.my_KalmannPlantNoPartials(X_in_minus_temp,u);
                
                J(:,count)=(f_plus-f_minus)/(2*delta_val);
            end
        end
        
        function Z = my_KalmannOutputNoPartials(obj,X_in)
            %Unpack the system state/system parameters
            theta=X_in(1);  %angle of rigid body with resepect to -y axis
            %Specifically, angle that line segment connecting pivot to
            %robot contact with respect to the -y axis
            
            dtheta_dt=X_in(2); %time derivative of theta
            
            a=X_in(3); %coefficient representing gravity and moment of inertia
            %for an arbitrary rigid body, a=mgl/I where l is the distance
            %of the center of mass from the pivot, and I is the moment of
            %inertia of the body with respect to the pivot
            %for a simple pendulum, a=3/2 g/l
            
            b=X_in(4); %coefficient representing the moment of inertia
            %about the pivot locaction.
            %for an arbitrary rigid body, b=1/I
            %for the simple pendulum systme, b=3/ml^2
            
            theta_0=X_in(5); %offset angle from ray1 to ray 2
            %where ray1 is the ray from pivot to center of mass
            %and ray2 is the ray from pivot to contact point
            
            x_c=X_in(6); %x coordinate of the pivot location in world frame
            y_c=X_in(7); %y coordinate of the pivot location in world frame
            R=X_in(8); %distance from pivot to contact point

            
            %a=(3/2) g/l
            %b=3/(ml^2)
            
            params.l=1;
            params.g=(2/3)*a;
            params.m=3/b;
            params.mu=obj.mu;
            params.t_m=obj.t_m;
            
            theta_in=theta-theta_0;
            
            obj.UpdateParams(params);
            
            obj.sticking_constraint_ground.UpdateParamsStickingContactOneBody([0;0],[x_c;y_c]);
            obj.MyEnvironment_constrained.assign_coordinate_vector([x_c;y_c;theta_in]);
            obj.MyEnvironment_constrained.assign_velocity_vector([0;0;dtheta_dt]);
            
            
            pin=R*[sin(theta_0);-cos(theta_0)];
            
            pout=obj.pendulum_rigid_body_object.rigid_body_position(pin);
            vout=obj.pendulum_rigid_body_object.rigid_body_velocity(pin);
            
            Z=[pout;vout];
        end
        
        function setPivot(obj,x_c,y_c)
            obj.sticking_constraint_ground.UpdateParamsStickingContactOneBody([0;0],[x_c;y_c]);            
            [~,theta]=obj.pendulum_rigid_body_object.get_p_and_theta();
            obj.pendulum_rigid_body_object.set_p_and_theta([x_c;y_c],theta);
        end
        
        function [Z,pZpX] = my_KalmannOutputWithPartials(obj,X_in)
            Z=obj.my_KalmannOutputNoPartials(X_in);
            
            delta_val=10^-6;
            pZpX=zeros(length(Z),length(X_in));

            for count=1:length(X_in)
                delta_X_in=zeros(length(X_in),1);
                delta_X_in(count)=delta_val;
                
                X_in_plus_temp=X_in+delta_X_in;
                X_in_minus_temp=X_in-delta_X_in;
                
                f_plus=obj.my_KalmannOutputNoPartials(X_in_plus_temp);
                f_minus=obj.my_KalmannOutputNoPartials(X_in_minus_temp);
                
                pZpX(:,count)=(f_plus-f_minus)/(2*delta_val);
            end
        end
        
        function [dXdt_guess,dPdt]= extended_kalmann_update(obj,Z,X_guess,u,P,Q,R)

            [dXdt_guess_star,F] = obj.my_KalmannPlantWithPartials(X_guess,u);
            [Z_guess,H] = obj.my_KalmannOutputWithPartials(X_guess);

            K=P*H'/R;
            dPdt=F*P+P*F'-K*H*P+Q;

            dXdt_guess=dXdt_guess_star+K*(Z-Z_guess);
        end
        
        
        %Updates the systems for the new parameter values
        function UpdateParams(obj,params)
            obj.m = params.m;       % mass  (kg)
            obj.l = params.l;       % length (m)
            obj.t_m = params.t_m;   % control torque limit (N*m)
            obj.g = params.g;                   % gravity (m/s^2)
            obj.mu = params.mu;     % coefficient of friction
            obj.I = obj.l^2 * obj.m^2/3;   % inertia (kg^2 * m^2)
            
            plist = [0,0;0,-obj.l];
            r_cm  = [0;-obj.l/2];
            I_com = obj.m*(obj.l)^2/12;
            
            
            obj.pendulum_rigid_body_object.UpdateParams(plist, r_cm, obj.m, I_com);
            obj.myGravity.UpdateParamsGravity([0;-obj.g]);            
        end

        
        % continuous forward dynamics (no partials)
        % [qkd, qkdd] = [qkd; M^{-1} * (B(q) * uk - c(qd))]
        % xk = [qk; qkd]
        % qk = [x_pivot; y_pivot; theta=angle w/respect negative y axis]
        % qkd = d/dt (qk)
        % uk = [lambda_x; lambda_y; tau]
        % fn and ft are forces exerted at the robot contact
        % fn and ft are defined in the contact frame
        % tau is a moment exerted on the object by the robot
        % lambda_x is force exerted in the x direction of the world frame
        % lambda_y is force exerted in the y direction of the world frame
        % lambda_x/lambda_y are exerted at the pivot of the object
        % f = d/dt xk
        function f = dynamics_no_partials(obj, xk, uk)
            obj.MyEnvironment.assign_coordinate_vector(xk(1:3));
            obj.MyEnvironment.assign_velocity_vector(xk(4:6));
            obj.ControlInput.set_wrench_value(uk);
            
            obj.MyEnvironment.computeAccelerations();
            
            a= obj.MyEnvironment.build_acceleration_vector();
            v= obj.MyEnvironment.build_velocity_vector();
            
            f=[v;a];
        end
        
        
        % continuous forward dynamics
        % [qkd, qkdd] = [qkd; M^{-1} * (B(q) * uk - c(qd))]
        % xk = [qk; qkd]
        % qk = [x_pivot; y_pivot; theta=angle w/respect negative y axis]
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
            f = obj.dynamics_no_partials(xk, uk);
            
            delta_val=10^-6;
            df_dx=zeros(length(f),length(xk));
            df_du=zeros(length(f),length(uk));
            for count=1:6
                delta_xk=zeros(6,1);
                delta_xk(count)=delta_val;
                
                xk_plus_temp=xk+delta_xk;
                xk_minus_temp=xk-delta_xk;
                
                f_plus=obj.dynamics_no_partials(xk_plus_temp,uk);
                f_minus=obj.dynamics_no_partials(xk_minus_temp,uk);
                
                df_dx(:,count)=(f_plus-f_minus)/(2*delta_val);
            end

            for count=1:3
                delta_uk=zeros(3,1);
                delta_uk(count)=delta_val;
                
                uk_plus_temp=uk+delta_uk;
                uk_minus_temp=uk-delta_uk;
                
                f_plus=obj.dynamics_no_partials(xk,uk_plus_temp);
                f_minus=obj.dynamics_no_partials(xk,uk_minus_temp);

                df_du(:,count)=(f_plus-f_minus)/(2*delta_val);
            end
            
        end
        
        % continuous forward dynamics (no partials) 
        %(for constrained system)
        % [qkd, qkdd] = [qkd; M^{-1} * (B(q) * uk - c(qd))]
        % xk = [qk; qkd]
        % qk = [x_pivot; y_pivot; theta=angle w/respect negative y axis]
        % qkd = d/dt (qk)
        % uk = [lambda_x; lambda_y; tau]
        % fn and ft are forces exerted at the robot contact
        % fn and ft are defined in the contact frame
        % tau is a moment exerted on the object by the robot
        % lambda_x is force exerted in the x direction of the world frame
        % lambda_y is force exerted in the y direction of the world frame
        % lambda_x/lambda_y are exerted at the pivot of the object
        % f = d/dt xk
        function f = constrained_dynamics_no_partials(obj, xk, uk)
            obj.MyEnvironment_constrained.assign_coordinate_vector(xk(1:3));
            obj.MyEnvironment_constrained.assign_velocity_vector(xk(4:6));
            obj.ControlInput.set_wrench_value(uk);
            
            obj.MyEnvironment_constrained.computeAccelerations();
            
            a= obj.MyEnvironment_constrained.build_acceleration_vector();
            v= obj.MyEnvironment_constrained.build_velocity_vector();
            
            f=[v;a];
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
            
            dc_dx = [zeros(obj.neq, obj.nq), [Dx;Dy]];
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
        
        % given xk, find xkp1 and uk (pivot forces; input torque) that
        % that satisfy the following equations:
        % (1) x_{k+1} = xk + dt * obj.dynamics(xk, uk)
        % (2) obj.pivot_const(xk) = 0
        % (3) fix torque (uk(end)) to the value given
        function [xkp1, uk_out] =  dynamics_solve(obj, xk, uk, dt)
            
            obj.MyEnvironment_constrained.assign_coordinate_vector(xk(1:3));
            obj.MyEnvironment_constrained.assign_velocity_vector(xk(4:6));
            obj.ControlInput.set_wrench_value([0;0;uk(3)]);
            
            obj.MyEnvironment_constrained.setdt(dt);
            
            obj.MyEnvironment_constrained.computeAccelerations();
                       

            obj.MyEnvironment_constrained.EulerUpdate();
            obj.MyEnvironment_constrained.ConstraintProjection();
            
            
            q=obj.MyEnvironment_constrained.build_coordinate_vector();
            q_dot=obj.MyEnvironment_constrained.build_velocity_vector();
            
            xkp1=[q;q_dot];
            uk_out=[obj.sticking_constraint_ground.getMultipliers();uk(3)];
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

