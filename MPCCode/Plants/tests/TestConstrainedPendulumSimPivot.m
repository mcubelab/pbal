% Simulate the ConstrainedRigidBodyPendulum

clear; clc; close all;
addpath('../')

% pendulum parameters
params.m = 1;           % mass
params.l = 1;           % length
params.g = 9.81;        % acceleration due to gravity
params.t_m = 0;         % torque limit
params.b = 0.1;         % damping

% plant
p = ConstrainedRigidBodyPendulumPivot(params);

% initial condition (note this has to satisfy pivot constraints)
thtk = pi/3;
xk = [0; 0; thtk; 0; 0; 0];
uk = zeros(p.nu, 1);  

% store state and inputs
xvec = xk;
uvec = [];

% store evaluation of equality const.
cvec = [];

dt = 0.01; % simulation time-step
N = 100; % simulation horizon
for i = 1:N
    
    % simulate one-step
    [xkp1, uk] = p.dynamics_solve(xk, uk, dt);      
    
    % store 
    xvec = [xvec, xkp1]; 
    uvec = [uvec, uk];
    cvec = [cvec, p.equality_const(xk, uk)];
    
    xk = xkp1; % update
end

save('constrained_pend_pivot', 'xvec', 'uvec', 'cvec')


%% Plotting 

t = (0:N)*dt; 

% state
figure(1); clf;
titles = {'x', 'y', 'tht', 'vx', 'vy', 'omega'};
multiplier = [1, 1, (180/pi), 1, 1, (180/pi)]; 
for i = 1:(p.nq + p.nv)    
    subplot(2, 3, i); hold on;
    plot(t, multiplier(i)*xvec(i, :));
    title(titles{i})
end

% input
figure(2); clf;
titles = {'fx', 'fy', 'tau'};
for i = 1:p.nq    
    subplot(1, 3, i); hold on;
    plot(t(1:end-1), uvec(i, :));
    title(titles{i})
end

% constraints
figure(3); clf;
titles = {'pivot-vx', 'pivot-vy'};
for i = 1:p.neq
    subplot(1, 2, i); hold on;
    plot(t(1:end-1), cvec(i, :));  
    title(titles{i});    
end

% % plot trajectory based on COM-XY and theta + pivot constraint
% figure(4); clf; hold on;
% plot(xvec(1, :), xvec(2, :))
% plot(0.5 * p.l * sin(xvec(3, :)), -0.5 * p.l * cos(xvec(3, :)), '--')
% axis equal;
% title('COM X-Y')
% legend('xy trajectory', 'theta trajectory')


