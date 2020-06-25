clear; clc; close all;

params.m = 0.1;
params.l = 0.2;
params.t_m = 0.5; 
p = Pendulum(params);

% upright fixed point
x0 = [0; 0; pi; 0; 0; 0]; %zeros(6, 1);
u0 = [0; p.m * p.g; 0];

% linearized dynamics
[LinearSystem.A, LinearSystem.B] = p.linearize_forward_dyn(x0, u0); 

% equality constraints
[Ax, bx] = p.build_state_const();
LinearSystem.E = Ax(:, 1:6);
LinearSystem.F = Ax(:, 7:9); 
LinearSystem.k = bx; 

% inequality constraints
[Au, bu] = p.build_input_const();
LinearSystem.G = Au(:, 1:6);
LinearSystem.J = Au(:, 7:9);
LinearSystem.l = bu; 

% build MPC
mpc = LinearMPC(LinearSystem, 20);

% set x0
mpc = mpc.set_nominal_state(0 * x0);

% set u0
mpc = mpc.set_nominal_control(0 * u0);

% set cost matrix
mpc = mpc.set_cost_matrix(blkdiag(10 * eye(mpc.nx/2), 0.01*eye(mpc.nx/2)), ...
    0.001*eye(mpc.nu)); 

% generate cost matrices
mpc = mpc.update_cost_mat();

% generate constraint matrices
mpc = mpc.update_constraint_mat();

% initial condition
xi = [0; 0; -pi/3; 0; 0; 0];

xvec = [];
uvec = [];


for i = 1:mpc.n
    [bigX, bigU, dt] = mpc.solve_qp_subproblem(xi); 
    ui = bigU(1:mpc.nu); 
    xi = LinearSystem.A*xi + LinearSystem.B*ui;
    z0 = [bigX; bigU]; 
    
    xvec = [xvec, xi]; 
    uvec = [uvec, ui];
end
%%

t = (0:(mpc.n-1))*p.h; 

figure(1); clf; 

subplot(2, 2, 1); hold on; 
plot(t, (180/pi) * (xvec(3, :)));
title('theta')

subplot(2, 2, 2); hold on;
plot(t, xvec(6, :));
title('omega')

subplot(2, 2, 3); hold on; 
plot(t, xvec(1:2, :));
title('x, y')
legend('x', 'y')

subplot(2, 2, 4); hold on; 
plot(t, xvec(4:5, :));
title('vx, vy')
legend('vx', 'vy')

figure(2); clf; 

subplot(1, 3, 1); hold on; 
plot(t, uvec(1, :));
title('Fx')

subplot(1, 3, 2); hold on;
plot(t, uvec(2, :));
title('Fy')

subplot(1, 3, 3); hold on; 
plot(t, uvec(3, :));
title('tau')



