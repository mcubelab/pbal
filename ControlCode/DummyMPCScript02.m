% Test MPC on pendulum stbalizaton about upright

clear; clc; close all;

params.m = 0.1;
params.l = 0.1;
params.dt = 0.005;
params.t_m = 0.1;
params.b = 0.001;
p = RigidBodyPendulum(params);

% upright fixed point
tht0 = pi;
x0 = [0.5 * p.l*sin(tht0); -0.5 * p.l*cos(tht0); tht0; 0; 0; 0];
u0 = [0; p.m * p.g; 0];

% linearized dynamics
[~, LinearSystem.A, LinearSystem.B] = p.forward_dyn_euler(x0, u0);

% equality constraints (TODO: why ONLY on velocity)
% E * xk + F  * uk = k
[~, dc_dx, dc_du] = p.pivot_const(x0);
LinearSystem.E = dc_dx((p.neq/2+1):end, :);
LinearSystem.F = dc_du((p.neq/2+1):end, :);
LinearSystem.k = zeros(p.neq/2, 1);

% inequality constraints
[Au, bu] = p.build_input_const();
LinearSystem.G = zeros(p.niq, p.nq + p.nv);
LinearSystem.J = Au;
LinearSystem.l = bu;

% build MPC
mpc = LinearMPC(LinearSystem, 50);

% set cost matrix
Q = blkdiag(10 * eye(mpc.nx/2), 0.01*eye(mpc.nx/2));
R = 0.001*eye(mpc.nu);
mpc = mpc.set_cost_matrix(Q, R);

% generate constraint matrices
mpc = mpc.update_constraint_mat();

% initial condition
thti = pi - pi/2;
xi = [0.5 * p.l*sin(thti); -0.5 * p.l*cos(thti); thti; 0; 0; 0];

dxi = xi - x0;
dxvec = dxi;
duvec = [];
tvec = [];


for i = 1:mpc.n
    
    % solves the MPC
    tic
    [bigX, bigU] = mpc.solve_qp_subproblem(dxi);
    tvec = [tvec, toc];
    dui = bigU(1:mpc.nu);
    
    % one step forward using MPC solution (not sure this is right)
    dxi = p.forward_dyn_euler(x0 + dxi, u0 + dui) - x0;
    
    dxvec = [dxvec, dxi];
    duvec = [duvec, dui];
end

xtrue = [x0 + dxvec(:, 1)];
utrue = [];
ctrue = [p.pivot_const(x0)]
for i = 1:mpc.n
    
    xi = xtrue(:, i);
    ui = u0 + duvec(:, i);
    xip1 = p.forward_dyn_euler(xi, ui);
    
    xtrue = [xtrue, xip1];
    utrue = [utrue, ui];
    ctrue = [ctrue, p.pivot_const(x0)]; 
    
end

%%

t = (0:(mpc.n))*p.dt;

% state
figure(1); clf;

titles = {'x', 'y', 'tht', 'vx', 'vy', 'omega'};
multiplier = [1, 1, (180/pi), 1, 1, (180/pi)]; 

for i = 1:(p.nq + p.nv)    
    subplot(2, 3, i); hold on;
    plot(t, multiplier(i)*(dxvec(i, :) + x0(i)));
    plot(t, multiplier(i)*xtrue(i, :), '--');
    title(titles{i})
end

figure(2); clf;


titles = {'fx', 'fy', 'tau'};

for i = 1:p.nq    
    subplot(1, 3, i); hold on;
    plot(t(1:end-1), duvec(i, :) + u0(i));
    plot(t(1:end-1), utrue(i, :), '--');
    title(titles{i})
end

figure(3); clf;

% subplot(1, 3, 1); hold on;
% plot(xtrue(1, :), xtrue(2, :));
% plot(0.5 * p.l * sin(xtrue(3, :)), -0.5 * p.l * cos(xtrue(3, :)), '--')

titles = {'pivot-x', 'pivot-y', 'pivot-vx', 'pivot-vy'};
for i = 1:p.neq
    subplot(2, 2, i); hold on;
    plot(t, ctrue(i, :));  
    title(titles{i});    
end

figure(4); clf; 
plot(t(1:end-1), 1./tvec, 'k*')
yline(mean(1./tvec), 'k')
title('MPC Frequency')

figure(5); clf; 
plot(xtrue(1, :), xtrue(2, :))
axis equal; 


