clear; clc; close all;

params.m = 0.1;
params.l = 0.1;
params.dt = 0.005; 
params.t_m = 0;
params.b = 0.001;  
p = RigidBodyPendulum(params);

thtk = pi/3;
xk = [0.5 * p.l*sin(thtk); -0.5 * p.l*cos(thtk); thtk; 0; 0; 0]; %zeros(6, 1);
uk = p.build_input_matrix(xk(1:3))\p.build_coriolis_and_potential(xk(4:6));
uk = uk.*[1; 1; 0]; 

xvec = [xk];
uvec = [];

c0 = p.pivot_const(xk); 
qpvec = [c0(1:p.neq/2)];
vpvec = [c0(p.neq/2:end)];

N = 100; 
for i = 1:N
    [xkp1, uk] = p.dynamics_solve(xk, uk);
    [ck] = p.pivot_const(xkp1); 
%     disp(xkp1)
    xvec = [xvec, xkp1]; 
    uvec = [uvec, uk];
    qpvec = [qpvec, ck(1:p.neq/2)];
    vpvec = [vpvec, ck(p.neq/2:end)]; 
    xk = xkp1;
end

%%
t = (0:N)*p.dt; 

figure(1); clf; 

subplot(2, 2, 1); hold on; 
plot(t, (180/pi) * xvec(3, :));
title('theta')


subplot(2, 2, 2); hold on;
plot(t, xvec(6, :));
title('omega')

subplot(2, 2, 3); hold on; 
plot(t, qpvec(1:2, :));
title('pivot_x, pivot_y')
legend('x', 'y')

subplot(2, 2, 4); hold on; 
plot(t, vpvec(1:2, :));
title('pivot_vx, pivot_vy')
legend('vx', 'vy')

figure(2); clf; 

subplot(1, 3, 1); hold on; 
plot(t(1:end-1), uvec(1, :));
title('Fx')

subplot(1, 3, 2); hold on;
plot(t(1:end-1), uvec(2, :));
title('Fy')

subplot(1, 3, 3); hold on; 
plot(t(1:end-1), uvec(3, :));
title('tau')

figure(3); hold on;
plot(xvec(1, :), xvec(2, :)); 
plot(0.5 * p.l * sin(xvec(3, :)), -0.5 * p.l * cos(xvec(3, :)), '--') 



