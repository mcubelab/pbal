clear; clc; close all;

nx = 3;
nu = 2; 
neq = 1;
niq = 1; 

% build linear system
LinearSystem.A = rand(nx);
LinearSystem.B = rand(nx, nu);

LinearSystem.E = 0 * rand(neq, nx);
LinearSystem.F = 0 * rand(neq, nu);

LinearSystem.G = 0 * rand(niq, nx);
LinearSystem.J = 0 * rand(niq, nu);

% build MPC
mpc = LinearMPC(LinearSystem, 10);

% set x0
mpc = mpc.set_nominal_state(zeros(nx, 1));

% set u0
mpc = mpc.set_nominal_control(zeros(nu, 1));

% set cost matrix
mpc = mpc.set_cost_matrix(eye(nx), 0.1*eye(nu)); 
mpc.R

% generate cost matrices
mpc = mpc.update_cost_mat();

% generate constraint matrices
mpc = mpc.update_constraint_mat();

xi = rand(3,1);
xi_uc = rand(3,1); 

xvec = [];
xvec_uc = []; 
uvec = [];
dtvec = [];

for i = 1:mpc.n
    [bigX, bigU, dt] = mpc.solve_qp_subproblem(xi); 
    ui = bigU(1:nu); 
    xi = LinearSystem.A*xi + LinearSystem.B*ui;
    xi_uc = LinearSystem.A*xi_uc;
    
    xvec = [xvec, xi]; 
    xvec_uc = [xvec_uc, xi_uc]; 
    uvec = [uvec, ui];
    dtvec = [dtvec; dt]; 
end

%%

figure; clf; 
subplot(2,1,1); hold on;
plot(1:mpc.n, xvec); 
% plot(1:mpc.n, xvec_uc, '--');
subplot(2,1,2); hold on;
plot(1:mpc.n, uvec); 


