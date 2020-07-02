% Simulate the pendulum

clear; clc; close all;

% inputs
params.m = 0.1;
params.l = 0.1;
params.dt = 0.005; 
params.t_m = 0;
params.b = 0.001;  
p = RigidBodyPendulum(params);

% initial condition (note this has to satisfy pivot constraints)
thtk = pi/3;
xk = [0.5 * p.l*sin(thtk); -0.5 * p.l*cos(thtk); thtk; 0; 0; 0];
uk = zeros(p.nu, 1);  

% store state and inputs
xvec = xk;
uvec = [];

% store evaluation of pivot const.
c0 = p.pivot_const(xk); 
cvec = c0;

% simulate for 100 steps
N = 100; 
for i = 1:N
    [xkp1, uk] = p.dynamics_solve(xk, uk);  % sim-step

    % store 
    xvec = [xvec, xkp1]; 
    uvec = [uvec, uk];
    cvec = [cvec, p.pivot_const(xkp1)];
    
    xk = xkp1; % update
end

%% Plotting 

t = (0:N)*p.dt; 

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
titles = {'pivot-x', 'pivot-y', 'pivot-vx', 'pivot-vy'};
for i = 1:p.neq
    subplot(2, 2, i); hold on;
    plot(t, cvec(i, :));  
    title(titles{i});    
end

% also constraints
figure(4); clf; hold on;
plot(xvec(1, :), xvec(2, :))
plot(0.5 * p.l * sin(xvec(3, :)), -0.5 * p.l * cos(xvec(4, :)), '--')
axis equal;
title('COM X-Y')


