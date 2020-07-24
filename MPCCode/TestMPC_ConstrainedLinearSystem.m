% % Test MPC with constrained linear system
clear; clc; close all;

% state/constraint dimensions
nx = 5;
nu = 4;
neq = 1;
niq = 2 * nu;

% random transition
ls_params.A = rand(nx) - 0.5;
ls_params.B = rand(nx, nu) - 0.5;

% random state constraint 
ls_params.Aeq = rand(neq, nx);
ls_params.Beq = 0 * eye(neq, nu);
ls_params.ceq = zeros(neq, 1);

% box input constraint
ls_params.Aiq = 0 * eye(niq, nx);
ls_params.Biq = [eye(niq/2, nu); -eye(niq/2, nu)];
ls_params.ciq = 0.3 * [ones(niq/2,1); ones(niq/2, 1)];
p = ConstrainedLinearSystem(ls_params);

% build MPC
mpc_params.Ntraj = 100;  % trajectory length
mpc_params.Nmpc = 20;   % mpc horizon
mpc_params.dt = 0.1;   % control rate

% cost matrices
mpc_params.QN = eye(p.nx);
mpc_params.Q = eye(p.nx);
mpc_params.R = 0.1*eye(p.nu);

% nominal trajectory
mpc_params.Xnom = repmat(zeros(nx,1), 1, mpc_params.Ntraj);
mpc_params.Unom = repmat(zeros(nu,1), 1, mpc_params.Ntraj);

% build mpc
mpc_tv = TimeVaryingMPC(p, mpc_params);

% initial state (has to be valid)
xk_guess = rand(nx, 1) - 0.5;
P = ls_params.Aeq' * ((ls_params.Aeq * ls_params.Aeq')\ls_params.Aeq);
xk = xk_guess - P * xk_guess;

% plotting
xvec = xk;
uvec = [];
cvec = [];

cmap = jet(mpc_tv.Ntraj);
for k = 1:mpc_tv.Ntraj
    
    [dx_mpc, dU_mpc] = mpc_tv.run_mpc(k, xk);
    uk = mpc_tv.Unom(:, k) + dU_mpc(1:mpc_tv.nu);
    xkp1 = xk + mpc_tv.dt * p.dynamics(xk, uk);
    
    % store solution
    xvec = [xvec, xkp1];
    uvec = [uvec, uk];
    cvec = [cvec, p.equality_const(xk, uk)];
    
    % update
    xk = xkp1;
end

%% Plotting

t = (0:(mpc_tv.Ntraj));

figure(1); clf;

subplot(1, 3, 1); hold on;
plot(t, xvec);
title('State')

subplot(1, 3, 2); hold on;
plot(t(1:end-1), uvec);
title('Input')

subplot(1, 3, 3); hold on;
plot(t(1:end-1), cvec);
title('Constraint')
