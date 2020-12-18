% Test MPC on pendulum stbalizaton about upright
clear; clc; close all;
addpath('./Plants', './Solvers', '../ControlCode/')

% physical parameters (MORE INTUITIVE TO TUNE)
mass = 0.25; % kg
length = 0.1; % m
gravity = 10; % m/s^2
inertia = mass*length^2;

% fixed parameters
mu_contact = 0.5;
mu_pivot = 1.0;
Nmax_pivot = 100;
Nmax_contact = 50;
l_contact = 0.25 * length;
contact_normal = [-1; 0];

%true parameters
x= pi/2 + pi/2*(rand(1) - 0.5);
dxdt= sqrt(gravity/length)*0.1*(rand()-.5);
a=mass*length*gravity;
b=1/inertia;
theta_0= 0;
x_c=0;
y_c=0;
R= 1.5 * length;
X=[x;dxdt]; %;a;b;theta_0;x_c;y_c;R];

fprintf('Initial position from pi/2: %f, \n Initial velocity: %f \r', x - pi/2, dxdt);

% initial guess parameters
x_guess= pi/2;
dxdt_guess = 0; %dxdt 
X_guess= [x_guess;dxdt_guess]; %;a_guess;b_guess;theta_0_guess;x_c_guess;y_c_guess;R_guess];

% true pendulum plant parameters
params.g=gravity;                   % gravity (m/s^2)
l_cm=params.g/(a*b);
params.m=a/(params.g*l_cm);        % mass  (kg)
params.I_cm = 0.1 * params.m * length^2;                     % moment of inertia about center of mass;
params.l_contact = l_contact;  % length of robot contact
params.mu_pivot= mu_pivot;     % coefficient of friction at obj/ground contact
params.mu_contact = mu_contact;   % coefficient of friction at obj/robot contact
params.Nmax_pivot = Nmax_pivot;   % maximum force the ground can exert on obj along contact normal
params.Nmax_contact = Nmax_contact;    % maximum force the robot can exert on obj along contact normal
params.contact_normal = contact_normal; % direction of the contact normal in the body frame
params.contact_point = R*[1;0];                   %location of contact point in the body frame
params.r_cm=l_cm*[cos(theta_0);sin(theta_0)];    %location of center of mass in the body frame
p = PyramidPlant01(params);
p.setPivot(x_c,y_c);

% % initial guess pendulum plant
params_guess = params;
p_guess = PyramidPlant01(params_guess);
p_guess.setPivot(x_c,y_c);                  % NEED TO CHANGE TO X_C_GUESS, Y_C_GUESS EVENTUALLY

% mpc parameters
mpc_params.Nmpc = 20;    % mpc horizon
mpc_params.Ntraj = 100;  % trajectory length
mpc_params.dt = 0.01;    % time-step
mpc_params.QN = blkdiag(eye(p.nq), length/gravity*eye(p.nv));
mpc_params.Q = blkdiag(eye(p.nq), length/gravity*eye(p.nv));
mpc_params.R = 1e-3*eye(p.nu);

% waypoint params
waypoint_params.tht_min = pi/12;            % minimum distance between waypoints
waypoint_params.omega_desired = pi/6;       % desired rotational velocity

% kalman filter parameters
R= 0.1*length*diag([1; 1; sqrt(gravity/length); sqrt(gravity/length)]); 
Q= 0.01*[sqrt(gravity/length), 0; 0, gravity/length];
P= 2 * [1, 0; 0, sqrt(gravity/length)];

xk = [x_c; y_c; x; 0; 0; dxdt]; % true initial state
xk_guess = [x_c; y_c; x_guess; 0; 0; dxdt_guess]; % guess initial state

xg = [x_c; y_c; pi/2 - theta_0; 0; 0; 0]; % goal state
ug = [0; 0; 0];  % goal input TODO: should not always be zero

% state/input to linearize about for mpc
mpc_params.x0 = xg;
mpc_params.u0 = ug;

% build mpc
mpc_wp = MPCWithWaypoints(waypoint_params, p_guess, mpc_params);

% plotting
xvec = xk;
xvec_guess = xk_guess;
X_guessvec = X_guess;
uvec = [];
lvec = [];
Pvec = P;

for k=1:mpc_wp.mpc_tv.Ntraj
    
    % compute control input
    tic;
    [dx_mpc, dU_mpc, exitflag] = mpc_wp.run_mpc_nearest_waypoint(xk_guess, true);
    if exitflag ~= 1
        fprintf('\r QP Solve failed, exitflag is: %d \r', exitflag)
        break      
    end
    uk = (mpc_wp.mpc_tv.u0 + dU_mpc(1:mpc_wp.mpc_tv.nu));
    dt1 = toc;
    
    % advance true state
    [xkp1, lk] = p.dynamics_solve(xk, uk, mpc_wp.mpc_tv.dt);
    
    %
    tic;
    Z = p.my_KalmannOutputNoPartials_state_only(X);  % observation (based on true system)
    
    % kalmann update
    [dXdt_guess,dPdt]= p_guess.extended_kalmann_update_state_only(Z,X_guess,...
        uk,P,Q,R);
    
    % advance guess x
    P=P+mpc_wp.mpc_tv.dt*dPdt;
    X_guess=X_guess+mpc_wp.mpc_tv.dt*dXdt_guess;
    xk_guess = [x_c; y_c; X_guess(1); 0; 0; X_guess(2)];
    dt2 = toc;
    
    % update system parameters
    %     p_guess.UpdateParams_kalmann_state_only(X_guess);
    
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
    X(1) = xkp1(3) + theta_0;
    X(2) = xkp1(6);
end


%% Plotting

t = 0:(size(xvec, 2)-1);
f1 = figure(1); clf; hold on;


% animation
s1 = subplot(1,2,1); hold on;
p.initialize_visualization(xvec(:, 1), uvec(:, 1), length/2)
t1 = title(sprintf('time: %f', 0.0));
set(s1, 'xlim', 1.25 * norm(params.contact_point) * [-1,1], ...
    'ylim', 1.25 * norm(params.contact_point) * [0,1])

s2 = subplot(1,2,2); hold on;
p_guess.initialize_visualization(xvec_guess(:, 1), uvec(:, 1), length/2)
t2 = title(sprintf('time: %f', 0.0));
set(s2, 'xlim', 2 * norm(params_guess.contact_point) * [-1, 1], ...
    'ylim', 2 * norm(params_guess.contact_point) * [0,1])


nskip = 1;
for i = 1:nskip:(numel(t)-1)
    
    p.update_visualization(xvec(:, i), uvec(:, i), length/2)
    set(s1, 'xlim', 1.25 * norm(params.contact_point) * [-1,1], ...
        'ylim', 1.25 * norm(params.contact_point) * [0,1])
    s1.Title.String = sprintf('time: %f', mpc_wp.mpc_tv.dt * t(i));
    %     print('-dpng', sprintf('./Animations/png/true_%3f.png',  mpc_tv.dt * t(i)))
    
    
    p_guess.update_visualization(xvec_guess(:, i), uvec(:, i), length/2)
    set(s2, 'xlim', 2 * norm(params_guess.contact_point) * [-1, 1], ...
        'ylim', 2 * norm(params_guess.contact_point) * [0,1])
    s2.Title.String = sprintf('time: %f', mpc_wp.mpc_tv.dt * t(i));
    %     print('-dpng', sprintf('./Animations/png/guess_%3f.png',  mpc_tv.dt * t(i)))
    
end

%% state estimates
figure(2); clf;

subplot(2, 1, 1); hold on;
shadedErrorBar(t, X_guessvec(1, :), sqrt(Pvec(1, 1:2:end)))
plot(t, xvec(3, :), 'r');       
title('theta')

subplot(2, 1, 2); hold on;
shadedErrorBar(t, X_guessvec(2, :), sqrt(Pvec(2, 2:2:end)))
plot(t, xvec(6, :), 'r');       
title('omega')

% input forces
figure(3); clf;
titles = {'f_x', 'f_y', 'tau'};
for k = 1:p.nq
    subplot(1, 3, k); hold on;
    p1 = plot(t(1:end-1), uvec(k, :));
    title(titles{k})
end

% recation forces
figure(4); clf;
titles = {'\lambda_x', '\lambda_y'};
for k = 1:2
    subplot(1, 2, k); hold on;
    p1 = plot(t(1:end-1), lvec(k, :));
    title(titles{k})
end

% velocity constraint
figure(5); clf;
titles = {'pivot-vx', 'pivot-vy'};
subplot(2, 1, 1); hold on;
plot(t, xvec(4, :));
title(titles{1});
subplot(2, 1, 2); hold on;
plot(t, xvec(5, :));
title(titles{2});

% tilefigs;
