clear; clc; close all;

nx = 3;
nu = 2; 
neq = 1;
niq = 1; 

% build linear system
LinearSystem.A = 0.1 * rand(nx);
LinearSystem.B = rand(nx, nu);

LinearSystem.E = rand(neq, nx);
LinearSystem.F = rand(neq, nu);

LinearSystem.G = rand(niq, nx);
LinearSystem.J = rand(niq, nu);

% build MPC
mpc = LinearMPC(LinearSystem, 10);

% set x0
mpc = mpc.set_nominal_state(zeros(nx, 1));

% set u0
mpc = mpc.set_nominal_control(zeros(nu, 1));

% set cost matrix
mpc = mpc.set_cost_matrix(eye(nx), 0.1*eye(nu)); 

% generate cost matrices
mpc = mpc.update_cost_mat();

% generate constraint matrices
mpc = mpc.update_constraint_mat();

xi = rand(3,1);

xvec = [];
uvec = [];
dtvec = [];

for i = 1:10
    [bigX, bigU, dt] = mpc.solve_qp_subproblem(xi); 
    ui = bigU(1:nu); 
    xi = LinearSystem.A*xi + LinearSystem.B*ui;
    
    xvec = [xvec, xi]; 
    uvec = [uvec, ui];
    dtvec = [dtvec; dt]; 
end

%%

figure; clf; 
subplot(2,1,1); hold on;
plot(1:mpc.n, xvec); 
subplot(2,1,2); hold on;
plot(1:mpc.n, uvec); 


