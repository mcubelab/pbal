clear; clc; close all;

params.m = 0.1;
params.l = 0.1;
params.dt = 0.005; 
params.t_m = 1;
params.b = 0.001;  
p = RigidBodyPendulum(params);


% upright fixed point
tht0 = pi;
x0 = [0.5 * p.l*sin(tht0); -0.5 * p.l*cos(tht0); tht0; 0; 0; 0];
u0 = [0; p.m * p.g; 0];

p.forward_dyn_euler(x0, u0)

% linearized dynamics
[~, LinearSystem.A, LinearSystem.B] = p.forward_dyn_euler(x0, u0);

% [A, B] = p.linearize_forward_dyn(x0, u0)

% equality constraints
[~, dc_dx, dc_du] = p.pivot_const(x0);
LinearSystem.E = dc_dx(3:4, :);
LinearSystem.F = dc_du(3:4, :); 
LinearSystem.k = zeros(p.neq/2, 1); 

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
thti = pi - pi/10;
xi = [0.5 * p.l*sin(thti); -0.5 * p.l*cos(thti); thti; 0; 0; 0];

dxi = xi - x0; 
dxvec = [dxi];
duvec = [];


for i = 1:mpc.n
    [bigX, bigU, dt] = mpc.solve_qp_subproblem(dxi); 
    dui = bigU(1:mpc.nu); 
    dxi = LinearSystem.A*dxi + LinearSystem.B * dui;
%     dxi = p.forward_dyn_euler(x0 + dxi, u0 + dui) - x0;
    z0 = [bigX; bigU]; 
    
    dxvec = [dxvec, dxi]; 
    duvec = [duvec, dui];
end

xtrue = [x0 + dxvec(:, 1)];
utrue = [];
for i = 1:mpc.n
    
    xi = xtrue(:, i); 
    ui = u0 + duvec(:, i); 
    xip1 = p.forward_dyn_euler(xi, ui); 
    
    xtrue = [xtrue, xip1];
    utrue = [utrue, ui]; 

end
    
%%

t = (0:(mpc.n))*p.dt; 

figure(1); clf; 

subplot(2, 2, 1); hold on; 
plot(t, (180/pi) * (dxvec(3, :)));
plot(t, (180/pi) * (xtrue(3, :) - x0(3)), '--');
title('theta')

subplot(2, 2, 2); hold on;
plot(t, dxvec(6, :));
plot(t, (180/pi) * (xtrue(6, :) - x0), '--');
title('omega')

subplot(2, 2, 3); hold on; 
plot(t, dxvec(1:2, :));
title('x, y')
legend('x', 'y')

subplot(2, 2, 4); hold on; 
plot(t, dxvec(4:5, :));
title('vx, vy')
legend('vx', 'vy')

figure(2); clf; 

subplot(2, 2, 1); hold on; 

title('theta')

subplot(2, 2, 2); hold on;
plot(t, xtrue(6, :));
title('omega')

subplot(2, 2, 3); hold on; 
plot(t, xtrue(1:2, :));
title('x, y')
legend('x', 'y')

subplot(2, 2, 4); hold on; 
plot(t, xtrue(4:5, :));
title('vx, vy')
legend('vx', 'vy')


figure(3); clf; 

subplot(1, 3, 1); hold on; 
plot(t(1:end-1), duvec(1, :));
title('Fx')

subplot(1, 3, 2); hold on;
plot(t(1:end-1), duvec(2, :));
title('Fy')

subplot(1, 3, 3); hold on; 
plot(t(1:end-1), duvec(3, :));
title('tau')

figure(4); clf; 

subplot(1, 3, 1); hold on; 
plot(t(1:end-1), utrue(1, :));
title('Fx')

subplot(1, 3, 2); hold on;
plot(t(1:end-1), utrue(2, :));
title('Fy')

subplot(1, 3, 3); hold on; 
plot(t(1:end-1), utrue(3, :));
title('tau')

figure(5); clf; 
subplot(1, 3, 1); hold on;
plot(xtrue(1, :), xtrue(2, :)); 
plot(0.5 * p.l * sin(xtrue(3, :)), -0.5 * p.l * cos(xtrue(3, :)), '--') 

subplot(1, 3, 2); hold on;
plot(t, xtrue(1, :)); 
plot(t, 0.5 * p.l * sin(xtrue(3, :)), '--') 

subplot(1, 3, 3); hold on;
plot(t, xtrue(2, :)); 
plot(t, -0.5 * p.l * cos(xtrue(3, :)), '--') 



