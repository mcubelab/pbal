clear; clc; close all; 

% Test MPC on pendulum stbalizaton about upright
clear; clc; close all;
addpath('./Plants', './Solvers', '../ControlCode/')


length = 0.1; % length scale (m)
theta0 = pi/12; 

% true pendulum plant parameters
params.g = 10;                   % gravity (m/s^2)
params.m = 0.25;                                         % mass  (kg)
params.I_cm = 0.1 * params.m * length^2;                     % moment of inertia about center of mass;
params.l_contact = 0.25 * length;  % length of robot contact
params.mu_pivot = 1.0;     % coefficient of friction at obj/ground contact
params.mu_contact = 0.5;   % coefficient of friction at obj/robot contact
params.Nmax_pivot = 100;   % maximum force the ground can exert on obj along contact normal
params.Nmax_contact = 5;    % maximum force the robot can exert on obj along contact normal
params.contact_normal = [-1; 0]; % direction of the contact normal in the body frame
params.contact_point = 1.5*length*[1;0];                   %location of contact point in the body frame
params.r_cm = length*[cos(theta0);sin(theta0)];    %location of center of mass in the body frame
p = PyramidPlant01(params);
p.setPivot(0,0);

% mpc parameters
mpc_params.Nmpc = 10;    % mpc horizon
mpc_params.Ntraj = 200;  % trajectory length
mpc_params.dt = 0.01;    % time-step
mpc_params.QN = blkdiag(eye(p.nq), 0.01*eye(p.nv));
mpc_params.Q = blkdiag(eye(p.nq), 0.01*eye(p.nv));
mpc_params.R = 0.001*eye(p.nu);

% waypoint params
waypoint_params.tht_min = pi/12;            % minimum distance between waypoints
waypoint_params.omega_desired = pi/6;       % desired rotational velocity

% initial state
xk = [0; 0; pi/2 - pi/3; 0; 0; 0]; 

xg = [0; 0; pi/2 - theta0; 0; 0; 0];                      % goal state
ug = [0; 0; 0];                                           % goal input

% state/input to linearize about for mpc
mpc_params.x0 = xg;
mpc_params.u0 = ug;

% build mpc
mpc_wp = MPCWithWaypoints(waypoint_params, p, mpc_params);

% plotting
xvec = xk;
uvec = [];
lvec = [];
dtvec = [];


% xk and xkp1 are in the format used in the mpc: [xp, yp, tht, vx_p, vy_p,
% omega];

% X and X_guess is in the format [tht_c; omega; a; b; tht_0; xp; yp; R];

for k=1:mpc_wp.mpc_tv.Ntraj
    
    % compute control input
    tic;
    [dx_mpc, dU_mpc] = mpc_wp.run_mpc_nearest_waypoint(xk, trtue);
    uk = (mpc_wp.mpc_tv.u0 + dU_mpc(1:mpc_wp.mpc_tv.nu));
    dt_mpc = toc;
    
    % advance true state
    [xkp1, lk] = p.dynamics_solve(xk, uk, mpc_wp.mpc_tv.dt);
   
    % print time
    fprintf('MPC Rate: %f \n', 1/dt_mpc);
    
    % store solution
    xvec = [xvec, xkp1];
    uvec = [uvec, uk];
    lvec = [lvec, lk];
    dtvec = [dtvec; dt_mpc]; 
    
    % update true x
    xk = xkp1;
end


%% Plotting

t = 0:(size(xvec, 2)-1);
f1 = figure(1); clf; hold on;

% animation
s1 = get(f1, 'currentaxes');
p.initialize_visualization(xvec(:, 1), uvec(:, 1), length/2)
t1 = title(sprintf('time: %f', 0.0));
axis equal; 
xlims = get(s1, 'xlim');
ylims = get(s1, 'ylim');

nskip = 1;
for i = 1:nskip:(numel(t)-1)
      
    p.update_visualization(xvec(:, i), uvec(:, i), length/2)
    set(s1, 'xlim', 1.25 * xlims, 'ylim', 1.25 * ylims)
    s1.Title.String = sprintf('time: %f', mpc_wp.mpc_tv.dt * t(i));
    pause(3*mpc_wp.mpc_tv.dt)
end

% state 
figure(3); clf;
titles = {'x', 'y', 'tht', 'vx', 'vy', 'omega'};
multiplier = [1, 1, (180/pi), 1, 1, (180/pi)];

for k = 1:(p.nq + p.nv)    
    subplot(2, 3, k); hold on;        
    plot(t, multiplier(k)*xvec(k, :), 'r'); % true state    
    title(titles{k})
end

% input forces
figure(4); clf;
titles = {'f_x', 'f_y', 'tau'};
for k = 1:p.nq
    subplot(1, 3, k); hold on;
    p1 = plot(t(1:end-1), uvec(k, :));
    title(titles{k})
end

% recation forces
figure(5); clf;
titles = {'\lambda_x', '\lambda_y'};
for k = 1:2
    subplot(1, 2, k); hold on;
    p1 = plot(t(1:end-1), lvec(k, :));
    title(titles{k})
end

% time
figure(7); clf; hold on;
title('MPC Frequency')
plot(1./dtvec);