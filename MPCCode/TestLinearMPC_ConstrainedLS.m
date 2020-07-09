% Test Linear MPC with constrained linear system

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

% set cost matrix
mpc = mpc.set_cost_matrix(10*eye(nx), eye(nx), 0.1*eye(nu)); 

% generate constraint matrices
mpc = mpc.update_qp_mat();

xi = 10 * (rand(3,1) - 0.5);
xi_uc = rand(3,1); 

xvec = [];
xvec_uc = []; 
uvec = [];

for i = 1:mpc.n
    [bigX, bigU] = mpc.solve_qp_subproblem(xi); 
    ui = bigU(1:nu); 
    xi = LinearSystem.A*xi + LinearSystem.B*ui;
    xi_uc = LinearSystem.A*xi_uc;
    
    xvec = [xvec, xi]; 
    xvec_uc = [xvec_uc, xi_uc]; 
    uvec = [uvec, ui];
end

%%

figure; clf; 
subplot(2,1,1); hold on;
plot(1:mpc.n, xvec); 
% plot(1:mpc.n, xvec_uc, '--');
subplot(2,1,2); hold on;
plot(1:mpc.n, uvec); 


