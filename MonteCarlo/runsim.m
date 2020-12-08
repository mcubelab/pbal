function [outputArg1,outputArg2] = runsim(x0,xguess,not_estimated_params, ...
    mpc_params, waypoint_params, kf_params)

% x0 is the initial state, which consists of 

% x0.x: angle of rigid body with resepect to +x axis
% Specifically, angle that line segment connecting pivot to
% robot contact with respect to the +x axis

% x0.dxdt: time derivative of theta
            
% x0.a: coefficient representing gravity and moment of inertia
% for an arbitrary rigid body, a=mgl/I where l is the distance
% of the center of mass from the pivot, and I is the moment of
% inertia of the body with respect to the pivot            
            
% x0.b: coefficient representing the moment of inertia
% about the pivot locaction.
% for an arbitrary rigid body, b=1/I            
            
% x0.theta_0: offset angle from ray1 to ray 2
% where ray1 is the ray from pivot to contact point
% and ray2 is the ray from pivot to center of mass
            
% x0.x_c: x coordinate of the pivot location in world frame
% x0.y_c: y coordinate of the pivot location in world frame
% x0.R: distance from pivot to contact point
            
% xhat is the guess state (mostly we will keep this constant)

% not_estimated_params contain parameters that are not estimated
% mu_contact: coefficient of friction b/w robot and object
% mu_pivot: coefficient of friction b/w object and surface
% Nmax_pivot: maximum normal force that surface can exert on object
% Nmax_contact: maximum normal force that robot can exert on object
% l_contact: length of contact patch b/w robot and object
% contact_normal: normal vector to contact patch b/w robot and object in
% the object frame
% gravity: gravity in m/s^2


% fixed parameters (from hyper parameters)
mu_contact = not_estimated_params.mu_contact;
mu_pivot = not_estimated_params.mu_pivot;
Nmax_pivot = not_estimated_params.Nmax_pivot;
Nmax_contact = not_estimated_params.Nmax_contact;
l_contact = not_estimated_params.l_contact;
contact_normal = not_estimated_params.contact_normal;
gravity = not_estimated_params.gravity; 
% Icm = not_estimated_params.Icm;

%true parameters
x = x0.x; 
dxdt = x0.dxdt;
a = x0.a; 
b = x0.b;
theta_0 = x0.theta_0;
x_c = x0.x_c;
y_c = x0.y_c;
R = x0.R;
X = [x;dxdt;a;b;theta_0;x_c;y_c;R];

% uses x, dxdt, mass, length, gravity, inertia, theta_0, xc_, y_c, and R

% initial guess parameters
x_guess= xguess.x;
dxdt_guess = xguess.dxdt;
a_guess = xguess.a;
b_guess = xguess.b;
theta_0_guess = xguess.theta_0;
x_c_guess = xguess.x_c;
y_c_guess = xguess.y_c;
R_guess = xguess.R;
X_guess = [x_guess;dxdt_guess;a_guess;b_guess;theta_0_guess; ...
    x_c_guess;y_c_guess;R_guess];


% true plant parameters
params.g = gravity;                                        % gravity (m/s^2)
l_cm = params.g/(a*b);                                     % distance b/w pivot and CM 
params.m = a/(params.g*l_cm);                              % mass  (kg)
params.I_cm = 0;                                           % moment of inertia about center of mass;
params.l_contact = l_contact;                              % length of robot contact
params.mu_pivot = mu_pivot;                                % coefficient of friction at obj/ground contact
params.mu_contact = mu_contact;                            % coefficient of friction at obj/robot contact
params.Nmax_pivot = Nmax_pivot;                            % maximum force the ground can exert on obj along contact normal
params.Nmax_contact = Nmax_contact;                        % maximum force the robot can exert on obj along contact normal
params.contact_normal = contact_normal;                    % direction of the contact normal in the body frame
params.contact_point = R*[1;0];                            % location of contact point in the body frame
params.r_cm = l_cm*[cos(theta_0);sin(theta_0)];            % location of center of mass in the body frame
p = PyramidPlant01(params);
p.setPivot(x_c,y_c);

% initial guess plant
params_guess = params; 
l_cm_guess = params_guess.g/(a_guess*b_guess);                             % distance b/w pivot and CM 
params_guess.m = a_guess/(params_guess.g*l_cm_guess);                      % mass  (kg)
params_guess.contact_point = R_guess*[1;0];                                % location of contact point in the body frame
params_guess.r_cm = l_cm_guess*[cos(theta_0_guess);sin(theta_0_guess)];    % location of center of mass in the body frame
p_guess = PyramidPlant01(params_guess);
p_guess.setPivot(x_c_guess,y_c_guess);

% mpc parameters
% mpc_params.Nmpc = 20;    % mpc horizon
% mpc_params.Ntraj = 300;  % trajectory length
% mpc_params.dt = 0.01;    % time-step
% mpc_params.QN = blkdiag(eye(p.nq), 0.01*eye(p.nv));
% mpc_params.Q = blkdiag(eye(p.nq), 0.01*eye(p.nv));
% mpc_params.R = 0.0001*eye(p.nu);

% waypoint params
waypoint_params.tht_min = pi/12;            % minimum distance between waypoints
waypoint_params.omega_desired = pi/6;       % desired rotational velocity

% kalman filter parameters
R = kf_params.R; %0.1*eye(4);
Q = kf_params.Q; %0.01*eye(8);
P = kf_paramd.P; %0.1*eye(8);

xk = [x_c; y_c; x; 0; 0; dxdt]; % true initial state
xk_guess = [x_c_guess; y_c_guess; x_guess; 0; 0; dxdt_guess]; % guess initial state

xg = [x_c_guess; y_c_guess; pi/2 - theta_0_guess; 0; 0; 0]; % goal state
% ug = 

% state/input to linearize about for mpc
mpc_params.x0 = xg;
mpc_params.u0 = ug;

% build mpc
mpc_wp = MPCWithWaypoints(waypoint_params, p, mpc_params);

% plotting
xvec = xk;
xvec_guess = xk_guess;
X_guessvec = X_guess;
uvec = [];
lvec = [];
Pvec = P;


% xk and xkp1 are in the format used in the mpc: [xp, yp, tht, vx_p, vy_p,
% omega];

% X and X_guess is in the format [tht_c; omega; a; b; tht_0; xp; yp; R];

for k=1:mpc_wp.mpc_tv.Ntraj
    
    % compute control input
    tic;
    [dx_mpc, dU_mpc] = mpc_wp.run_mpc_nearest_waypoint(xk_guess, true);
    uk = (mpc_wp.mpc_tv.u0 + dU_mpc(1:mpc_wp.mpc_tv.nu));
    dt1 = toc;
    
    % advance true state
    [xkp1, lk] = p.dynamics_solve(xk, uk, mpc_wp.mpc_tv.dt);
    
    %
    tic;
    Z = p.my_KalmannOutputNoPartials(X);  % observation
    
    % kalmann update
    [dXdt_guess,dPdt]= p_guess.extended_kalmann_update(Z,X_guess,...
        uk,P,Q,R);
    
    % advance guess x
    P=P+mpc_wp.mpc_tv.dt*dPdt;
    X_guess=X_guess+mpc_wp.mpc_tv.dt*dXdt_guess;
    xk_guess = [X_guess(6); X_guess(7); X_guess(1); 0; 0; X_guess(2)];
    dt2 = toc;
    
    % update system parameters
    p_guess.UpdateParams_kalmann(X_guess);
    
    fprintf('\r MPC Rate: %f,  KF Rate: %f, Total Rate: %f', ...
        1/dt1, 1/dt2, 1/(dt1 + dt2))
    
    % store solution
    xvec = [xvec, xkp1];
    xvec_guess = [xvec_guess, xk_guess];
    uvec = [uvec, uk];
    lvec = [lvec, lk];
    X_guessvec = [X_guessvec, X_guess];
    Pvec = [Pvec, P];
    
    % update true x
    xk = xkp1;
    X(1) = xkp1(3) + X(5);
    X(2) = xkp1(6);
end


%% Plotting
% 
% t = 0:(size(xvec, 2)-1);
% f1 = figure(1); clf; hold on;
% 
% 
% % animation
% s1 = subplot(1,2,1); hold on;
% p.initialize_visualization(xvec(:, 1), uvec(:, 1), length/2)
% t1 = title(sprintf('time: %f', 0.0));
% set(s1, 'xlim', 1.25 * norm(params.contact_point) * [-1,1], ...
%     'ylim', 1.25 * norm(params.contact_point) * [0,1])
% 
% s2 = subplot(1,2,2); hold on;
% p_guess.initialize_visualization(xvec_guess(:, 1), uvec(:, 1), length/2)
% t2 = title(sprintf('time: %f', 0.0));
% set(s2, 'xlim', 2 * norm(params_guess.contact_point) * [-1, 1], ...
%     'ylim', 2 * norm(params_guess.contact_point) * [0,1])
% 
% 
% nskip = 1;
% for i = 1:nskip:(numel(t)-1)
%     
%     p.update_visualization(xvec(:, i), uvec(:, i), length/2)
%     set(s1, 'xlim', 1.25 * norm(params.contact_point) * [-1,1], ...
%         'ylim', 1.25 * norm(params.contact_point) * [0,1])
%     s1.Title.String = sprintf('time: %f', mpc_wp.mpc_tv.dt * t(i));
%     %     print('-dpng', sprintf('./Animations/png/true_%3f.png',  mpc_tv.dt * t(i)))
%     
%     
%     p_guess.update_visualization(xvec_guess(:, i), uvec(:, i), length/2)
%     set(s2, 'xlim', 2 * norm(params_guess.contact_point) * [-1, 1], ...
%         'ylim', 2 * norm(params_guess.contact_point) * [0,1])
%     s2.Title.String = sprintf('time: %f', mpc_wp.mpc_tv.dt * t(i));
%     %     print('-dpng', sprintf('./Animations/png/guess_%3f.png',  mpc_tv.dt * t(i)))
%     
% end
% 
% % state estimates
% figure(3); clf;
% 
% titles = {'x', 'y', 'tht', 'vx', 'vy', 'omega'};
% multiplier = [1, 1, (180/pi), 1, 1, (180/pi)];
% 
% for k = 1:(p.nq + p.nv)
%     
%     subplot(2, 3, k); hold on;
%     
%     % guess state + uncertainty
%     if k == 1  % theta
%         shadedErrorBar(t, multiplier(k)*X_guessvec(6, :), ...
%             multiplier(k)*sqrt(Pvec(6, 6:8:end)))
%     end
%     
%     if k == 2  % x_c
%         shadedErrorBar(t, multiplier(k)*X_guessvec(7, :), ...
%             multiplier(k)*sqrt(Pvec(7, 7:8:end)))
%     end
%     
%     if k == 3 % y_c
%         shadedErrorBar(t, multiplier(k)*X_guessvec(1, :), ...
%             multiplier(k)*sqrt(Pvec(1, 1:8:end)))
%     end
%     if k == 6 % omega
%         shadedErrorBar(t, multiplier(k)*X_guessvec(2, :), ...
%             multiplier(k)*sqrt(Pvec(2, 2:8:end)))
%     end
%     
%     % true state
%     plot(t, multiplier(k)*xvec(k, :), 'r');
%     
%     title(titles{k})
% end
% 
% % parameter estimates
% figure(4); clf;
% subplot(3,1,1); hold on;
% shadedErrorBar(t, X_guessvec(3, :), sqrt(Pvec(3, 3:8:end)))
% yline(a, 'r')
% title('Gravity-ish term')
% subplot(3,1,2); hold on;
% shadedErrorBar(t, X_guessvec(4, :), sqrt(Pvec(4, 4:8:end)))
% yline(b, 'r')
% title('Mass-ish term')
% subplot(3,1,3); hold on;
% shadedErrorBar(t, X_guessvec(5, :), sqrt(Pvec(5, 5:8:end)))
% yline(theta_0, 'r')
% title('Theta_0')
% 
% % input forces
% figure(5); clf;
% titles = {'f_x', 'f_y', 'tau'};
% for k = 1:p.nq
%     subplot(1, 3, k); hold on;
%     p1 = plot(t(1:end-1), uvec(k, :));
%     title(titles{k})
% end
% 
% % recation forces
% figure(6); clf;
% titles = {'\lambda_x', '\lambda_y'};
% for k = 1:2
%     subplot(1, 2, k); hold on;
%     p1 = plot(t(1:end-1), lvec(k, :));
%     title(titles{k})
% end
% 
% % velocity constraint
% figure(7); clf;
% titles = {'pivot-vx', 'pivot-vy'};
% subplot(2, 1, 1); hold on;
% plot(t, xvec(4, :));
% title(titles{1});
% subplot(2, 1, 2); hold on;
% plot(t, xvec(5, :));
% title(titles{2});
% 
% % tilefigs;

end

