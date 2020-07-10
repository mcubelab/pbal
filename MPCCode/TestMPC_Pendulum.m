% Test MPC on pendulum stbalizaton about upright
clear; clc; close all; 

traj_scale=1;

pendulum_params.m = 0.1;
pendulum_params.l = 0.1;
pendulum_params.dt = 0.005/traj_scale;
pendulum_params.t_m = 0.05;
pendulum_params.b = 0.001;
p = RigidBodyPendulum(pendulum_params);

% goal state
thtg = pi/3;
xg = [0.5 * p.l*sin(thtg); -0.5 * p.l*cos(thtg); thtg; 0; 0; 0];
ug = [0; p.m * p.g; 0.5 * p.m * p.g * p.l*sin(thtg)];

% state at time-step k
thtk = thtg - pi/3;
xk = [0.5 * p.l*sin(thtk); -0.5 * p.l*cos(thtk); thtk; 0; 0; 0];
% uk = [0; p.m * p.g; 0.5 * p.m * p.g * p.l*sin(thtk)];

% build MPC
mpc_params.Ntraj = 50*traj_scale;  % trajectory length 
mpc_params.Nmpc = 5*traj_scale;   % mpc horizon

% cost matrices
mpc_params.QN = blkdiag(eye(p.nq), 0.01*eye(p.nv)); 
mpc_params.Q = blkdiag(eye(p.nq), 0.01*eye(p.nv));
mpc_params.R = 0.001*eye(p.nu);

% nominal trajectory
mpc_params.Xnom = repmat(xg, 1, mpc_params.Ntraj);
mpc_params.Unom = repmat(ug, 1, mpc_params.Ntraj); 
% mpc_params.Xnom = xk + linspace(0, 1, mpc_params.Ntraj).*(xg - xk);  
% mpc_params.Unom = uk + linspace(0, 1, mpc_params.Ntraj).*(ug - uk);

% build mpc
mpc_tv = TimeVaryingMPC(p, mpc_params);

% plotting
xvec = xk;
uvec = [];
cvec = [p.pivot_const(xk)];

cmap = jet(mpc_tv.Ntraj); 
for k = 1:mpc_tv.Ntraj


    tic;
    [dx_mpc, dU_mpc] = mpc_tv.run_mpc(k, xk);
    disp(toc); 
    uk = mpc_tv.Unom(:, k) + dU_mpc(1:mpc_tv.nu); 
    xkp1 = xk + p.dt*p.dynamics(xk, uk);
    disp(xkp1')

    % store solution
    xvec = [xvec, xkp1];
    uvec = [uvec, uk]; 
    cvec = [cvec, p.pivot_const(xkp1)];

    % update
    xk = xkp1; 
end

% xtrue = [xg + dxvec(:, 1)];
% utrue = [];
% ctrue = [p.pivot_const(xg)]
% for k = 1:mpc_tv.n
%     
%     xk = xtrue(:, k);
%     ui = ug + duvec(:, k);
%     xip1 = xtrue(:, k) + p.dt * p.dynamics(xk, ui);
%     
%     xtrue = [xtrue, xip1];
%     utrue = [utrue, ui];
%     ctrue = [ctrue, p.pivot_const(xg)]; 
%     
% end

%% Plotting


t = (0:(mpc_tv.Ntraj))*p.dt;

% state
figure(1); clf;

titles = {'x', 'y', 'tht', 'vx', 'vy', 'omega'};
multiplier = [1, 1, (180/pi), 1, 1, (180/pi)]; 

for k = 1:(p.nq + p.nv)    
    subplot(2, 3, k); hold on;
    plot(t, multiplier(k)*xvec(k, :));
    title(titles{k})
end

figure(2); clf;


titles = {'fx', 'fy', 'tau'};

for k = 1:p.nq    
    subplot(1, 3, k); hold on;
    plot(t(1:end-1), uvec(k, :));
    title(titles{k})
end

figure(3); clf;

titles = {'pivot-x', 'pivot-y', 'pivot-vx', 'pivot-vy'};
for k = 1:p.neq
    subplot(2, 2, k); hold on;
    plot(t, cvec(k, :));  
    title(titles{k});    
end
% 
% figure(4); clf; 
% plot(t(1:end-1), 1./tvec, 'k*')
% yline(mean(1./tvec), 'k')
% title('MPC Frequency')
% 
% figure(5); clf; 
% plot(xtrue(1, :), xtrue(2, :))
% axis equal; 


% set(0,'DefaultFigureWindowStyle','normal')
% 
% titles = {'x', 'y', 'tht', 'vx', 'vy', 'omega'};
% multiplier = [1, 1, (180/pi), 1, 1, (180/pi)];
% 
% figure(3);
% for i = 1:(p.nq + p.nv)
%     subplot(2, 3, i); hold on;
%     plot(t, multiplier(i) * xvec(i, :), '--');
%     %         plot(t, multiplier(i)*xtrue(i, :), '--');
%     title(titles{i})
% end
% 
% 
% 
% figure(4);
% titles = {'fx', 'fy', 'tau'};
% 
% for i = 1:p.nq
%     subplot(1, 3, i); hold on;
%     plot(t(1:end-1), uvec(i, :), '--');
%     %         plot(t(1:end-1), utrue(i, :), '--');
%     title(titles{i})
% end

% figure(5); clf;
% 
% % subplot(1, 3, 1); hold on;
% % plot(xtrue(1, :), xtrue(2, :));
% % plot(0.5 * p.l * sin(xtrue(3, :)), -0.5 * p.l * cos(xtrue(3, :)), '--')
% 
% titles = {'pivot-x', 'pivot-y', 'pivot-vx', 'pivot-vy'};
% for k = 1:p.neq
%     subplot(2, 2, k); hold on;
%     plot(t, ctrue(k, :));
%     title(titles{k});
% end

%     xtrue = [xg + dxvec(:, 1)];
%     utrue = [];
%     ctrue = [p.pivot_const(xg)]
%     for i = 1:mpc.n
%
%         x0 = xtrue(:, i);
%         ui = ug + duvec(:, i);
%         xip1 = p.forward_dyn_euler(x0, ui);
%
%         xtrue = [xtrue, xip1];
%         utrue = [utrue, ui];
%         ctrue = [ctrue, p.pivot_const(xg)];
%
%     end

%%



% figure(4); clf;
% plot(t(1:end-1), 1./tvec, 'k*')
% yline(mean(1./tvec), 'k')
% title('MPC Frequency')
%
% figure(5); clf;
% plot(xtrue(1, :), xtrue(2, :))
% axis equal;


