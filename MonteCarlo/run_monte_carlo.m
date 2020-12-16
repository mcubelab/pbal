clear; clc; close all; 
addpath('../','../ControlCode/', '../MPCCode/Plants', ...
    '../MPCCode/Solvers')

% NOMINAL SYSTEM
mass = 0.25; % kg
length = 0.1; % m
gravity = 10; % m/s^2
inertia = mass*length^2;

% fixed parameters (from hyper parameters)
not_estimated_params.mu_contact = 0.5;
not_estimated_params.mu_pivot = 1.0;
not_estimated_params.Nmax_pivot = 100;
not_estimated_params.Nmax_contact = 5;
not_estimated_params.l_contact = 0.25 * length;
not_estimated_params.contact_normal = [-1; 0];
not_estimated_params.gravity = 10; 

%true parameters
x0.x = pi/4*(rand(1) - 0.5) + pi/2; 
x0.dxdt = 0;
x0.a = mass*length*gravity*(0.5*(rand(1) - 0.5) + 1); 
x0.b = 1/inertia*(0.5*(rand(1) - 0.5) + 1);
x0.theta_0 = 0*(rand(1) - 0.5);
x0.x_c = 0*length*(rand(1) - 0.5);
x0.y_c = 0*length*(rand(1) - 0.5);
x0.R = length; %2*length*rand(1);

% initial guess parameters
xguess.x= pi/2;
xguess.dxdt = 0;
xguess.a = mass*length*gravity;
xguess.b = 1/inertia;
xguess.theta_0 = 0;
xguess.x_c = 0;
xguess.y_c = 0;
xguess.R = length;

% mpc parameters
mpc_params.Nmpc = 20;    % mpc horizon
mpc_params.Ntraj = 100;  % trajectory length
mpc_params.dt = 0.01;    % time-step
mpc_params.QN = blkdiag(10 * eye(3), 0.01*eye(3));
mpc_params.Q = blkdiag(10 * eye(3), 0.01*eye(3));
mpc_params.R = 0.0001*eye(3);

% waypoint params
waypoint_params.tht_min = pi/12;            % minimum distance between waypoints
waypoint_params.omega_desired = pi/6;       % desired rotational velocity

% kalman filter parameters
kf_params.R = 0.1*eye(4);
kf_params.Q = 0.01*eye(8);
kf_params.P = 0.1*eye(8);

[is_feasible, succeed, solution, p, p_guess] = runsim(x0,xguess,not_estimated_params, ...
    mpc_params, waypoint_params, kf_params);

fprintf('Initial Guess Feasible: %d \r', is_feasible)
fprintf('Algorithm Successful: %d \r', succeed)

%% Plotting

t = solution.t;
xvec = solution.xvec;
xvec_guess = solution.xvec_guess;
uvec = solution.uvec;
lvec = solution.lvec;
X_guessvec = solution.X_guessvec;
Pvec = solution.Pvec;

% t = 0:(size(xvec, 2)-1);
f1 = figure(1); clf; hold on;

% animation
s1 = subplot(1,2,1); hold on;
p.initialize_visualization(xvec(:, 1), uvec(:, 1), length/2)
t1 = title(sprintf('time: %f', 0.0));
set(s1, 'xlim', 1.25 * norm(p.contact_point) * [-1,1], ...
    'ylim', 1.25 * norm(p.contact_point) * [0,1])

s2 = subplot(1,2,2); hold on;
p_guess.initialize_visualization(xvec_guess(:, 1), uvec(:, 1), length/2)
t2 = title(sprintf('time: %f', 0.0));
set(s2, 'xlim', 2 * norm(p_guess.contact_point) * [-1, 1], ...
    'ylim', 2 * norm(p_guess.contact_point) * [0,1])


nskip = 1;
for i = 1:nskip:(numel(t)-1)
    
    p.update_visualization(xvec(:, i), uvec(:, i), length/2)
    set(s1, 'xlim', 1.25 * norm(p.contact_point) * [-1,1], ...
        'ylim', 1.25 * norm(p.contact_point) * [0,1])
%     s1.Title.String = sprintf('time: %f', mpc_wp.mpc_tv.dt * t(i));   
    
    p_guess.update_visualization(xvec_guess(:, i), uvec(:, i), length/2)
    set(s2, 'xlim', 2 * norm(p_guess.contact_point) * [-1, 1], ...
        'ylim', 2 * norm(p_guess.contact_point) * [0,1])
%     s2.Title.String = sprintf('time: %f', mpc_wp.mpc_tv.dt * t(i));
    
end

% state estimates
figure(3); clf;

titles = {'x', 'y', 'tht', 'vx', 'vy', 'omega'};
multiplier = [1, 1, (180/pi), 1, 1, (180/pi)];

for k = 1:(p.nq + p.nv)
    
    subplot(2, 3, k); hold on;
    
    % guess state + uncertainty
    if k == 1  % theta
        shadedErrorBar(t, multiplier(k)*X_guessvec(6, :), ...
            multiplier(k)*sqrt(Pvec(6, 6:8:end)))
    end
    
    if k == 2  % x_c
        shadedErrorBar(t, multiplier(k)*X_guessvec(7, :), ...
            multiplier(k)*sqrt(Pvec(7, 7:8:end)))
    end
    
    if k == 3 % y_c
        shadedErrorBar(t, multiplier(k)*X_guessvec(1, :), ...
            multiplier(k)*sqrt(Pvec(1, 1:8:end)))
    end
    if k == 6 % omega
        shadedErrorBar(t, multiplier(k)*X_guessvec(2, :), ...
            multiplier(k)*sqrt(Pvec(2, 2:8:end)))
    end
    
    % true state
    plot(t, multiplier(k)*xvec(k, :), 'r');
    
    title(titles{k})
end

% parameter estimates
figure(4); clf;
subplot(3,1,1); hold on;
shadedErrorBar(t, X_guessvec(3, :), sqrt(Pvec(3, 3:8:end)))
yline(x0.a, 'r');
title('Gravity-ish term')
subplot(3,1,2); hold on;
shadedErrorBar(t, X_guessvec(4, :), sqrt(Pvec(4, 4:8:end)))
yline(x0.b, 'r');
title('Mass-ish term')
subplot(3,1,3); hold on;
shadedErrorBar(t, X_guessvec(5, :), sqrt(Pvec(5, 5:8:end)))
yline(x0.theta_0, 'r');
title('Theta_0')

% input forces
figure(5); clf;
titles = {'f_x', 'f_y', 'tau'};
for k = 1:p.nq
    subplot(1, 3, k); hold on;
    p1 = plot(t(1:end-1), uvec(k, :));
    title(titles{k})
end

% recation forces
figure(6); clf;
titles = {'\lambda_x', '\lambda_y'};
for k = 1:2
    subplot(1, 2, k); hold on;
    p1 = plot(t(1:end-1), lvec(k, :));
    title(titles{k})
end

% velocity constraint
figure(7); clf;
titles = {'pivot-vx', 'pivot-vy'};
subplot(2, 1, 1); hold on;
plot(t, xvec(4, :));
title(titles{1});
subplot(2, 1, 2); hold on;
plot(t, xvec(5, :));
title(titles{2});

tilefigs; 



