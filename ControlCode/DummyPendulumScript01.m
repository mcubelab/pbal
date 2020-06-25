clear; clc; close all;

params.m = 1;
params.l = 0.1;
params.t_m = 0;
p = Pendulum(params);

xk = [0; 0; pi/3; 0; 0; 0]; %zeros(6, 1);
uk = zeros(3, 1);

xvec = [xk];
uvec = [];

for i = 1:100
    [xkp1, uk] = p.dynamics_solve(xk, uk);
    xvec = [xvec, xkp1]; 
    uvec = [uvec, uk];
    xk = xkp1;
end


t = (0:100)*p.h; 

figure(1); clf; 

subplot(2, 2, 1); hold on; 
plot(t, (180/pi) * xvec(3, :));
title('theta')


t = (0:100)*p.h; 

figure(1); clf; 


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
plot(t(1:end-1), uvec(1, :));
title('Fx')

subplot(1, 3, 2); hold on;
plot(t(1:end-1), uvec(2, :));
title('Fy')

subplot(1, 3, 3); hold on; 
plot(t(1:end-1), uvec(3, :));
title('tau')



