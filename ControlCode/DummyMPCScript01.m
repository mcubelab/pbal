clear; clc; close all;

nx = 3;
nu = 2; 
neq = 1;
niq = 4; 

% build linear system
LinearSystem.A = rand(nx);
LinearSystem.B = rand(nx, nu);

LinearSystem.E = 0*eye(neq, nx);
LinearSystem.F = 0*eye(neq, nu);
LinearSystem.k = 0*zeros(neq, 1); 

LinearSystem.G = 0*eye(niq, nx);
LinearSystem.J = 0 * [-eye(nu); eye(nu)];
LinearSystem.l = 0.*[ones(nu, 1); ones(nu, 1)];  

% build MPC
mpc = LinearMPC(LinearSystem, 30);

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

xi = 10 * (rand(3,1) - 0.5);
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


