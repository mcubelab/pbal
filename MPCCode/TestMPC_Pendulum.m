% Simulate the pendulum

clear; clc; close all;

% inputs
params.m = 1;
params.l = 1;
params.g = 9.81;
params.t_m = params.m * params.g * params.l/2;
params.b = 0;  
p = RigidBodyPendulum(params);

% build MPC
mpc_params.Ntraj = 500;  % trajectory length
mpc_params.Nmpc = 30;   % mpc horizon
mpc_params.dt = 0.01; 


% cost matrices
mpc_params.QN = 10 * diag([1, sqrt(p.I/(p.m * p.g * p.l))]); %eye(p.nx);
mpc_params.Q = diag([1, sqrt(p.I/(p.m * p.g * p.l))]);
mpc_params.R = 0.01*eye(p.nu);

% nominal trajectory
mpc_params.Xnom = repmat([pi; 0], 1, mpc_params.Ntraj);
mpc_params.Unom = repmat(zeros(p.nu,1), 1, mpc_params.Ntraj);

% build mpc
mpc_tv = TimeVaryingMPC(p, mpc_params);

% initial condition
xk = [2*pi*(rand(1) - 0.5); 0];

% plotting
xvec = xk;
uvec = [];

for k = 1:mpc_tv.Ntraj
    
    [dx_mpc, dU_mpc] = mpc_tv.run_mpc(k, xk);
    uk = mpc_tv.Unom(:, k) + dU_mpc(1:mpc_tv.nu);
    xkp1 = xk + mpc_tv.dt * p.dynamics(xk, uk);
    
    % store solution
    xvec = [xvec, xkp1];
    uvec = [uvec, uk];
    
    % update
    xk = xkp1;
end

%% Plotting

t = (0:(mpc_tv.Ntraj));

% animate
figure(1); clf; 
hold on;
xlim([-1, 1])
ylim([-1, 1])
ph = plot([0; p.l*sin(xvec(1,1))], [0; -p.l*cos(xvec(1,1))]);
th = title(sprintf('time: %f', 0.0)); 
for i = 1:mpc_tv.Ntraj
   set(ph, 'Xdata', [0; p.l*sin(xvec(1,i))], ...
       'Ydata', [0; -p.l*cos(xvec(1,i))])
   set(th, 'string', sprintf('time: %f', t(i)*mpc_tv.dt))
   pause(mpc_tv.dt)
end

figure(2); clf; hold on;

% state
subplot(1, 2, 1); hold on;
plot(t, xvec);
title('State')

% input
subplot(1, 2, 2); hold on;
plot(t(1:end-1), uvec);
title('Input')


