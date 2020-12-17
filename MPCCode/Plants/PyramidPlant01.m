classdef PyramidPlant01 < handle
    % A rod of mass (m) and length(l) pivoting about its top end in the
    % gravity plane
    
    properties
        
        % user specified
        m;   % mass (kg)
        r_cm; %location of center of mass in the body frame
        
        I_cm; %moment of inertia about center of mass;
        %         t_m  % control torque limit (N * m)
        
        
        % fixed/derived
        g;              % acceleration due to gravity (kg/m^2)
        fmincon_opt;    % options for fmincon
        
        mu_pivot;       % coefficient of friction at obj/ground contact
        mu_contact;     % coefficient of friction at obj/robot contact
        Nmax_pivot;     % maximum force the ground can exert on obj along contact normal
        Nmax_contact;   % maximum force the robot can exert on obj along contact normal
        contact_normal; % direction of the contact normal in the body frame
        contact_point;  % location of contact point in the body frame
        
        l_contact;          % length of robot contact
        
        % dimensions
        nq;                 % config dimension
        nv;                 % gen gevelocity dimension
        nx;
        nu;                 % input dimension
        neq;                % # of equality const
        niq;                % # of inequality const
        
        %Simulation Environment instance representing this system
        %In this case, there is no pivot constraint
        %In this case, the system is constrained
        MyEnvironment;
        
        %Rigid body representing the pendulum in the simulation
        pendulum_rigid_body_object;
        
        %Generalized force representing the wrench of the end effector
        EffectorWrench;
        
        %Constraint associated with ground contact
        sticking_constraint_ground;
        
        %gravity
        myGravity;
        
        
        %Plotting objects for the various contact forces and friction cone
        %boundaries
        Contact_force_draw;
        Contact_generator1_draw;
        Contact_generator2_draw;
        Ground_force_draw;
        Ground_generator1_draw;
        Ground_generator2_draw;
        
    end
    
    methods(Static)
        
        % TODO: This is probably defined in the wrong place
        function J = jacobian(contant_point)
            
            rx = contant_point(1);
            ry = contant_point(2);
            rt = 0;  % TODO: fix this --> generalize to contact frame not aligned with reference fame
            
            J = [cos(rt) -sin(rt), 0;
                sin(rt) cos(rt), 0;
                rx*sin(rt) - ry*cos(rt), rx*cos(rt) + ry*sin(rt), 1];
        end
        
    end
    
    methods
        
        % initialize
        function obj = PyramidPlant01(params)
            
            obj.m = params.m;       % mass  (kg)
            obj.I_cm = params.I_cm; % moment of inertia about center of mass;
            
            obj.g = params.g;                     % gravity (m/s^2)
            obj.mu_pivot = params.mu_pivot;       % coefficient of friction at obj/ground contact
            obj.mu_contact = params.mu_contact;   % coefficient of friction at obj/robot contact
            obj.Nmax_pivot = params.Nmax_pivot;   % maximum force the ground can exert on obj along contact normal
            obj.Nmax_contact = params.Nmax_contact; % maximum force the robot can exert on obj along contact normal
            obj.l_contact = params.l_contact;     % length of object/robot contact
            obj.contact_normal = params.contact_normal; % direction of the contact normal in the body frame
            
            obj.contact_point = params.contact_point;   %location of contact point in the body frame
            obj.r_cm  = params.r_cm;            %location of center of mass in the body frame
            
            
            plist = [[0;0],obj.contact_point];
            
            obj.pendulum_rigid_body_object=PolygonRigidBody(plist, obj.r_cm, obj.m, obj.I_cm);
            
            obj.sticking_constraint_ground=PolygonConstraint();
            obj.sticking_constraint_ground.StickingContactOneBody(obj.pendulum_rigid_body_object,[0;0],[0;0]);
            
            obj.myGravity=PolygonGeneralizedForce();
            obj.myGravity.gravity(obj.pendulum_rigid_body_object,[0;-obj.g]);
            
            obj.EffectorWrench=PolygonGeneralizedForce();
            
            obj.EffectorWrench.external_wrench(obj.pendulum_rigid_body_object,obj.contact_point);
            
            obj.MyEnvironment=SimulationEnvironment();
            
            obj.MyEnvironment.addRigidBody(obj.pendulum_rigid_body_object);
            obj.MyEnvironment.addConstraint(obj.sticking_constraint_ground);
            obj.MyEnvironment.addGeneralizedForce(obj.myGravity);
            
            obj.MyEnvironment.addGeneralizedForce(obj.EffectorWrench);
            
            % dimensions
            obj.nq = 3;
            obj.nv = 3;
            obj.nx = obj.nq + obj.nv;
            obj.nu = 3;
            obj.neq = 2;
            obj.niq = 4;
        end
        
        
        function dXdt = my_KalmannPlantNoPartials(obj,X_in,u)
            [~,xk] = obj.UpdateParams_kalmann(X_in);
            
            f = obj.dynamics_no_partials(xk, u);
            
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
        
        %Unpack the system state/system parameters
        %and inject them into the system
        function [params,xk] = UpdateParams_kalmann(obj, X_in)
            %Unpack the system state/system parameters
            theta=X_in(1);  %angle of rigid body with resepect to +x axis
            %Specifically, angle that line segment connecting pivot to
            %robot contact with respect to the +x axis
            
            dtheta_dt=X_in(2); %time derivative of theta
            
            a=X_in(3); %coefficient representing gravity and moment of inertia
            %for an arbitrary rigid body, a=mgl/I where l is the distance
            %of the center of mass from the pivot, and I is the moment of
            %inertia of the body with respect to the pivot
            
            
            b=X_in(4); %coefficient representing the moment of inertia
            %about the pivot locaction.
            %for an arbitrary rigid body, b=1/I
            
            
            theta_0=X_in(5); %offset angle from ray1 to ray 2
            %where ray1 is the ray from pivot to contact point
            %and ray2 is the ray from pivot to center of mass
            
            x_c=X_in(6); %x coordinate of the pivot location in world frame
            y_c=X_in(7); %y coordinate of the pivot location in world frame
            R=X_in(8); %distance from pivot to contact point
            
            
            params.l_contact=obj.l_contact;
            
            params.g=10;
            
            %m*g*l_cm = a -> m*l_cm=a/g
            %m*lcm^2+I_cm =I_pivot= 1/b -> m*lcm^2=(m*lcm)*lcm=1/b
            %l_cm*a/g=1/b-> g/(ab)=l_cm
            
            
            l_cm=params.g/(a*b);  %distance from pivot to the center of mass
            
            params.I_cm=0;
            params.m=a/(params.g*l_cm);
            
            
            params.contact_point= R*[1;0];
            params.r_cm= l_cm*[cos(theta_0);sin(theta_0)];
            
            
            
            obj.UpdateParams(params);
            
            
            obj.sticking_constraint_ground.UpdateParamsStickingContactOneBody([0;0],[x_c;y_c]);
            obj.MyEnvironment.assign_coordinate_vector([x_c;y_c;theta]);
            obj.MyEnvironment.assign_velocity_vector([0;0;dtheta_dt]);
            
            xk=[x_c;y_c;theta;0;0;dtheta_dt];
        end
        
        function initialize_visualization(obj, xk, uk, length_scale)
            
            obj.MyEnvironment.assign_coordinate_vector(xk(1:3));
            obj.MyEnvironment.assign_velocity_vector(xk(4:6));
            obj.EffectorWrench.set_wrench_value(uk);
            
            obj.MyEnvironment.computeAccelerations();
            
            pivot_multipliers=obj.sticking_constraint_ground.getMultipliers();
            
            obj.pendulum_rigid_body_object.initialize_visualization();
            
            pivot_WF=obj.pendulum_rigid_body_object.rigid_body_position([0;0]);
            contact_WF=obj.pendulum_rigid_body_object.rigid_body_position(obj.contact_point);
            
            Ground_generator1_WF=[obj.mu_pivot,1];
            Ground_generator2_WF=[-obj.mu_pivot,1];
            
            Contact_generator1_BF=obj.mu_contact*([0,1;-1,0]*obj.contact_normal)+obj.contact_normal;
            Contact_generator2_BF=-obj.mu_contact*([0,1;-1,0]*obj.contact_normal)+obj.contact_normal;
            
            Contact_generator1_WF=obj.pendulum_rigid_body_object.rigid_body_vector(Contact_generator1_BF);
            Contact_generator2_WF=obj.pendulum_rigid_body_object.rigid_body_vector(Contact_generator2_BF);
            
            Contact_generator1_plot_vector=length_scale*Contact_generator1_WF;
            Contact_generator2_plot_vector=length_scale*Contact_generator2_WF;
            
            Ground_generator1_plot_vector=length_scale*Ground_generator1_WF;
            Ground_generator2_plot_vector=length_scale*Ground_generator2_WF;
            
            Contact_force_plot_vector=length_scale*[uk(1);uk(2)]; %/obj.Nmax_contact;
            Ground_force_plot_vector=length_scale*pivot_multipliers; %/obj.Nmax_contact;
            
            
            obj.Contact_force_draw =line('XData',contact_WF(1)+[0,Contact_force_plot_vector(1)],...
                'YData',contact_WF(2)+[0,Contact_force_plot_vector(2)],...
                'color',[0, 179, 239]/255,'linewidth',2);
            
            obj.Contact_generator1_draw=line('XData',contact_WF(1)+[0,Contact_generator1_plot_vector(1)],...
                'YData',contact_WF(2)+[0,Contact_generator1_plot_vector(2)],...
                'color',[0, 179, 239]/255,'linewidth',2, 'linestyle', '--');
            obj.Contact_generator2_draw=line('XData',contact_WF(1)+[0,Contact_generator2_plot_vector(1)],...
                'YData',contact_WF(2)+[0,Contact_generator2_plot_vector(2)],...
                'color',[0, 179, 239]/255,'linewidth',2, 'linestyle', '--');
            
            obj.Ground_force_draw=line('XData',pivot_WF(1)+[0,Ground_force_plot_vector(1)],...
                'YData',pivot_WF(2)+[0,Ground_force_plot_vector(2)],...
                'color',[241, 90, 41]/255,'linewidth',2);
            
            obj.Ground_generator1_draw=line('XData',pivot_WF(1)+[0,Ground_generator1_plot_vector(1)],...
                'YData',pivot_WF(2)+[0,Ground_generator1_plot_vector(2)],...
                'color',[241, 90, 41]/255,'linewidth',2,'linestyle', '--');
            obj.Ground_generator2_draw=line('XData',pivot_WF(1)+[0,Ground_generator2_plot_vector(1)],...
                'YData',pivot_WF(2)+[0,Ground_generator2_plot_vector(2)],...
                'color',[241, 90, 41]/255,'linewidth',2,'linestyle', '--');
            
            
        end
        
        function update_visualization(obj, xk, uk, length_scale)
            
            obj.MyEnvironment.assign_coordinate_vector(xk(1:3));
            obj.MyEnvironment.assign_velocity_vector(xk(4:6));
            obj.EffectorWrench.set_wrench_value(uk);
            obj.MyEnvironment.computeAccelerations();
            pivot_multipliers=obj.sticking_constraint_ground.getMultipliers();
            
            obj.pendulum_rigid_body_object.update_visualization();
            
            pivot_WF=obj.pendulum_rigid_body_object.rigid_body_position([0;0]);
            contact_WF=obj.pendulum_rigid_body_object.rigid_body_position(obj.contact_point);
            
            Ground_generator1_WF=[obj.mu_pivot,1];
            Ground_generator2_WF=[-obj.mu_pivot,1];
            
            Contact_generator1_BF=obj.mu_contact*([0,1;-1,0]*obj.contact_normal)+obj.contact_normal;
            Contact_generator2_BF=-obj.mu_contact*([0,1;-1,0]*obj.contact_normal)+obj.contact_normal;
            
            Contact_generator1_WF=obj.pendulum_rigid_body_object.rigid_body_vector(Contact_generator1_BF);
            Contact_generator2_WF=obj.pendulum_rigid_body_object.rigid_body_vector(Contact_generator2_BF);
            
            Contact_generator1_plot_vector=length_scale*Contact_generator1_WF;
            Contact_generator2_plot_vector=length_scale*Contact_generator2_WF;
            
            Ground_generator1_plot_vector=length_scale*Ground_generator1_WF;
            Ground_generator2_plot_vector=length_scale*Ground_generator2_WF;
            
            Contact_force_plot_vector=length_scale*[uk(1);uk(2)]; %/obj.Nmax_contact;
            Ground_force_plot_vector=length_scale*pivot_multipliers; %/obj.Nmax_contact;
            
            
            set(obj.Contact_force_draw ,'XData',contact_WF(1)+[0,Contact_force_plot_vector(1)],...
                'YData',contact_WF(2)+[0,Contact_force_plot_vector(2)]);
            
            set(obj.Contact_generator1_draw,'XData',contact_WF(1)+[0,Contact_generator1_plot_vector(1)],...
                'YData',contact_WF(2)+[0,Contact_generator1_plot_vector(2)]);
            set(obj.Contact_generator2_draw,'XData',contact_WF(1)+[0,Contact_generator2_plot_vector(1)],...
                'YData',contact_WF(2)+[0,Contact_generator2_plot_vector(2)]);
            
            set(obj.Ground_force_draw,'XData',pivot_WF(1)+[0,Ground_force_plot_vector(1)],...
                'YData',pivot_WF(2)+[0,Ground_force_plot_vector(2)]);
            
            set(obj.Ground_generator1_draw,'XData',pivot_WF(1)+[0,Ground_generator1_plot_vector(1)],...
                'YData',pivot_WF(2)+[0,Ground_generator1_plot_vector(2)]);
            set(obj.Ground_generator2_draw,'XData',pivot_WF(1)+[0,Ground_generator2_plot_vector(1)],...
                'YData',pivot_WF(2)+[0,Ground_generator2_plot_vector(2)]);
            
            drawnow;
            
        end
        
        function Z = my_KalmannOutputNoPartials(obj,X_in)
            [params,~] = obj.UpdateParams_kalmann(X_in);
            
            
            pin=params.contact_point;
            
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
            %             obj.PrintParams();
            
            [dXdt_guess_star,F] = obj.my_KalmannPlantWithPartials(X_guess,u);
            [Z_guess,H] = obj.my_KalmannOutputWithPartials(X_guess);
            
            K=P*H'/R;
            dPdt=F*P+P*F'-K*H*P+Q;
            
            dXdt_guess=dXdt_guess_star+K*(Z-Z_guess);
        end
        
        %Prints the system paramaters
        function PrintParams(obj)
            'Mass'
            obj.m
            'I_cm'
            obj.I_cm
            'contact point'
            obj.contact_point
            'r_cm'
            obj.r_cm
            
        end
        
        %Updates the systems for the new parameter values
        function UpdateParams(obj,params)
            
            obj.m = params.m;       % mass  (kg)
            obj.I_cm = params.I_cm; % moment of inertia about center of mass;
            %             obj.t_m = params.t_m;   % control torque limit (N*m)
            obj.g = params.g;       % gravity (m/s^2)
            
            
            obj.contact_point = params.contact_point;   %location of contact point in the body frame
            obj.r_cm  = params.r_cm;            %location of center of mass in the body frame
            
            plist = [[0;0],obj.r_cm];
            
            obj.EffectorWrench.set_wrench_location(obj.contact_point);
            obj.pendulum_rigid_body_object.UpdateParams(plist, obj.r_cm, obj.m, obj.I_cm);
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
            obj.EffectorWrench.set_wrench_value(uk);
            
            obj.MyEnvironment.computeAccelerations();
            
            a= obj.MyEnvironment.build_acceleration_vector();
            v= obj.MyEnvironment.build_velocity_vector();
            
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
            obj.EffectorWrench.set_wrench_value(uk);
            
            % TODO this doesn't look right
            c= obj.pendulum_rigid_body_object.rigid_body_velocity([0;0]);
            [~,~,Dx,Dy,~,~]=obj.pendulum_rigid_body_object. ...
                rigid_body_position_derivatives([0;0]);
            
            dc_dx = [zeros(obj.neq, obj.nq), [Dx;Dy]];
            dc_du = zeros(2, length(uk));
            
        end
        
        % inequality constraints, c(x, u) <= 0, and first derivatives
        % figure this one out later
        function c = inequality_const_no_partials(obj, xk, uk)
            
            
%             obj.MyEnvironment.assign_coordinate_vector(xk(1:3));
%             obj.MyEnvironment.assign_velocity_vector(xk(4:6));
%             obj.EffectorWrench.set_wrench_value(uk);     

            % RECOMPUTING LAGRANGE multipliers based on xk and uk
            obj.dynamics(xk, uk);
            
            % compute multipliers as a function of xk and uk
            pivot_multipliers=obj.sticking_constraint_ground.getMultipliers();
            lambda_x = pivot_multipliers(1);
            lambda_y = pivot_multipliers(2);
            
            % [lambda_y >=0; lambda_y <= Nmax_pivot; lambda_x <= -mu_pivot
            % * lambda_y; -mu_pivot * lambda_y >= lambda_x];
            pivot_force_constraint = [-lambda_y; lambda_y; lambda_x - obj.mu_pivot*lambda_y; -lambda_x - obj.mu_pivot*lambda_y] ...
                - [0; obj.Nmax_pivot; 0; 0];
            
            % contact wrench constraint in the world frame
            contact_wrench_constraint = obj.line_wrench_cone_constraints(xk, uk);
            
            % all inequality constraints
            c = [pivot_force_constraint; contact_wrench_constraint];
            
        end
        
        function [c, dc_dx, dc_du] = inequality_const(obj, xk, uk)
            
            c = obj.inequality_const_no_partials(xk, uk);
            
            Ix = eye(numel(xk));
            Iu = eye(numel(uk));
            EPS = 1e-6;
            
            dc_dx = zeros(numel(c), numel(xk));
            for i = 1:numel(xk)
                cp =  obj.inequality_const_no_partials(xk + Ix(:,i)*EPS, uk);
                cm =  obj.inequality_const_no_partials(xk - Ix(:,i)*EPS, uk);
                dc_dx(:,i) = (cp - cm)/(2*EPS);
            end
            
            dc_du = zeros(numel(c), numel(uk));
            for i = 1:numel(uk)
                cp =  inequality_const_no_partials(obj, xk, uk + Iu(:,i)*EPS);
                cm =  inequality_const_no_partials(obj, xk, uk - Iu(:,i)*EPS);
                dc_du(:,i) = (cp - cm)/(2*EPS);
            end
        end
        
        
        
        % given xk, find xkp1 and uk (pivot forces; input torque) that
        % that satisfy the following equations:
        % (1) x_{k+1} = xk + dt * obj.dynamics(xk, uk)
        % (2) obj.pivot_const(xk) = 0
        % (3) fix torque (uk(end)) to the value given
        function [xkp1, pivot_multipliers] =  dynamics_solve(obj, xk, uk, dt)
            
            obj.MyEnvironment.assign_coordinate_vector(xk(1:3));
            obj.MyEnvironment.assign_velocity_vector(xk(4:6));
            obj.EffectorWrench.set_wrench_value(uk);
            
            obj.MyEnvironment.computeAccelerations();
            
            obj.MyEnvironment.setdt(dt);
            obj.MyEnvironment.EulerUpdate();
            obj.MyEnvironment.ConstraintProjection();
            
            
            q=obj.MyEnvironment.build_coordinate_vector();
            q_dot=obj.MyEnvironment.build_velocity_vector();
            
            xkp1=[q;q_dot];
            pivot_multipliers=obj.sticking_constraint_ground.getMultipliers();
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
        
        function c = line_wrench_cone_constraints(obj, xk, uk)
            
%             test = [1; 0; 0];
            
%             fc = obj.Nmax_contact*[1.0, 1.0; -obj.mu_contact, obj.mu_contact; 0, 0];    % friction cone at at end point of line
%             
%             plp = [0; obj.l_contact/2; 0];             % pos end of line (contact frame)
%             plm = [0; -obj.l_contact/2; 0];            % neg end of line (contact frame)
%             
%             wcp = PyramidPlant01.jacobian(plp)*fc;         % wrench at line COM from top
%             wcm = PyramidPlant01.jacobian(plm)*fc;         % wrench at line COM from bottom
%             wc = [wcp, wcm(:,2), wcm(:,1)];                % generators for wrench cone in contact frame
            
            % compute the normals for the sides of the wrench cone
%             wc_wrap = [wc, wc(:,1)];
%             outward_facing_normals_contact_frame = zeros(3, 4);
            
            b = obj.l_contact/2;
            a = obj.mu_contact;
            
            % normals are the columns
            outward_facing_normals_contact_frame = -[2*b, 0, -2;
                2*a, 2, 0;
                2*b, 0, 2;
                2*a, -2, 0; 
                -1, 0, 0]'; 
                
                
             
%             for i = 1:(size(wc_wrap,2) - 1)
%                 outward_facing_normals_contact_frame(:,i) = -cross(wc_wrap(:,i), ...
%                     wc_wrap(:,i+1));
%             end
%             
%             
            % rotation from contact to body and body to world
            RContactToBody = blkdiag([obj.contact_normal, ...
                PolygonMath.theta_to_rotmat(pi/2)*obj.contact_normal], 1);
            RBodyToWorld = blkdiag(PolygonMath.theta_to_rotmat(xk(3)), 1);
            
            % generator for wrench at contact in world frame
%             wcw = RBodyToWorld*RContactToBody*wc;
            
%             test_world =  RBodyToWorld*RContactToBody*test;
            
%             % compute the normals for the sides of the wrench cone
%             wcw_wrap = [wcw, wcw(:,1)];
%             outward_facing_normals_world_frame = zeros(3, 4);
%             
%             for i = 1:(size(wcw_wrap,2) - 1)
%                 outward_facing_normals_world_frame(:,i) = -cross(wcw_wrap(:,i), ...
%                     wcw_wrap(:,i+1));
%             end

            % outward facing normals for sides of wrench cone in the world frame
            outward_facing_normals_world_frame = RBodyToWorld*RContactToBody*...
                outward_facing_normals_contact_frame;
            
            % compute the normal for the top
%             outward_facing_normals_world_frame = [outward_facing_normals_world_frame, ...
%                 [PolygonMath.theta_to_rotmat(xk(3))*obj.contact_normal; 0]];
            
            b = [0; 0; 0; 0; obj.Nmax_contact];
            
            c = outward_facing_normals_world_frame'*uk - b;
            
        end
        
        
    end
end

