% Test MPC on pendulum stbalizaton about upright
clear; clc; close all;
addpath('./Plants', './Solvers', '../ControlCode/')

% velocity desired
VD = pi/6; % radians per second

% pendulum parameters
pparams.m = 1; % mass
pparams.l = 1; % length
pparams.g = 9.81; % acceleration due to gravity
pparams.t_m = pparams.m * pparams.g * pparams.l; % torque limit on input
pparams.b = 0.0;  % damping
pparams.mu = 0.3; % coefficient of friction
p = PendulumPlant01(pparams);

p2 = ConstrainedRigidBodyPendulumPivot(pparams);

% mpc parameters
mpc_params.Nmpc = 20;    % mpc horizon
mpc_params.dt = 0.01;    % time-step
mpc_params.QN = blkdiag(eye(p.nq), 0.1*eye(p.nv));
mpc_params.Q = blkdiag(eye(p.nq), 0.1*eye(p.nv));
mpc_params.R = 0.001*eye(p.nu);

thtk = pi - pi/3;    % initial angle
xk = [0; 0; thtk; 0; 0; 0]; % initial state

thtgf = pi;  % final angle
dtht_g = pi/18;
thtg_vec = thtk:(dtht_g):thtgf; % intermediate goal states

% plottingself
xvec = xk;
uvec = [];
utruevec = [];
cvec = [];

for i = 1:numel(thtg_vec)
    
    
    thtg = thtg_vec(i); 
    xg = [0; 0; thtg; 0; 0; VD];
    ug = [0; p.m * p.g; 0.5 * p.m * p.g * p.l*sin(thtg)];
    disp(rad2deg(thtg))
    
    % traj length
    disp(i)
    mpc_params.Ntraj = ceil(1.2 * dtht_g/VD/mpc_params.dt);
    if i == ( numel(thtg_vec))
         disp('final traj');
         mpc_params.Ntraj = 2 * ceil(dtht_g/VD/mpc_params.dt);
         xg = [0; 0; thtg; 0; 0; 0];
         ug = [0; p.m * p.g; 0.5 * p.m * p.g * p.l*sin(thtg)];
    end

    % nominal trajectory
    mpc_params.Xnom = repmat(xg, 1, mpc_params.Ntraj);
    mpc_params.Unom = repmat(ug, 1, mpc_params.Ntraj);

    % build mpc
    mpc_tv = TimeVaryingMPC(p, mpc_params);

    for k = 1:mpc_tv.Ntraj

        [dx_mpc, dU_mpc] = mpc_tv.run_mpc(k, xk);
        uk = mpc_tv.Unom(:, k) + dU_mpc(1:mpc_tv.nu);
        [xkp1, uk_true] = p.dynamics_solve(xk, uk, mpc_tv.dt);

        % store solution
        xvec = [xvec, xkp1];
        uvec = [uvec, uk];
        utruevec = [utruevec, uk_true];
        cvec = [cvec, p.equality_const(xk, uk)];

        % update
        xk = xkp1;
    end
end

save('constrained_pend_pivot', 'xvec', 'uvec', 'utruevec', 'cvec'); 


%% Plotting

t = 0:(size(xvec, 2)-1); 
disp(t(end) * mpc_tv.dt); 
% t = (0:(mpc_tv.Ntraj));

% animation
figure(1); clf;
hold on; axis equal;
xlim([-1.5, 1.5])
ylim([-1.5, 1.5])
ph = plot( [0; xvec(1, 1) + p.l * sin(xvec(3,1))], ...
    [0; xvec(2, 1) - p.l * cos(xvec(3,1))]);
th = title(sprintf('time: %f', 0.0));
for i = 1:numel(t)
    set(ph, 'Xdata',  [0; xvec(1, i) + p.l * sin(xvec(3,i))], ...
        'Ydata', [0; 2 * xvec(2, i) -  p.l * cos(xvec(3,i))])
    set(th, 'string', sprintf('time: %f', t(i)*mpc_tv.dt))
    pause(mpc_tv.dt)
end

% state
figure(2); clf;

titles = {'x', 'y', 'tht', 'vx', 'vy', 'omega'};
multiplier = [1, 1, (180/pi), 1, 1, (180/pi)];

for k = 1:(p.nq + p.nv)
    subplot(2, 3, k); hold on;
    plot(t, multiplier(k)*xvec(k, :));
    title(titles{k})
end

% input
figure(3); clf;
titles = {'fx', 'fy', 'tau'};
for k = 1:p.nq
    subplot(1, 3, k); hold on;
    p1 = plot(t(1:end-1), uvec(k, :));
    p2 = plot(t(1:end-1), utruevec(k, :), '--');
    title(titles{k})
%     legend(p1, p2, 'Linear Estimate', 'True Force'); 
end

% velocity constraint
figure(4); clf;
titles = {'pivot-vx', 'pivot-vy'};
for k = 1:p.neq
    subplot(2, 1, k); hold on;
    plot(t(1:end-1), cvec(k, :));
    title(titles{k});
end