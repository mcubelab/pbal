% Test MPC on pendulum stbalizaton about upright
clear; clc; close all;
addpath('./Plants', './Solvers', '../ControlCode/')

% true parmeters
x= pi - pi/6;
dxdt=0;
a=7;
b=5;
theta_0=0;
x_c=1;
y_c=-2;
R=8;
X=[x;dxdt;a;b;theta_0;x_c;y_c;R];

% inital guess
x_guess= x+pi/4;
dxdt_guess=0.2;
a_guess=7-1;
b_guess=5+1;
theta_0_guess=pi/6;
x_c_guess=1-0.2;
y_c_guess=-2+0.2;
R_guess=8-1;
X_guess=[x_guess;dxdt_guess;a_guess;b_guess;theta_0_guess;x_c_guess;y_c_guess;R_guess];
% X_guess=X;

% kalman pendulum plant
params_guess.l = 1; % length
params_guess.g= (2/3)*a_guess;
params_guess.m= 3/b_guess;
params_guess.t_m = params_guess.m * params_guess.g * params_guess.l; % torque limit on input
params_guess.b = 0.0;  % damping
params_guess.mu = 0.3; % coefficient of friction
p_guess = PendulumPlant01(params_guess);
p_guess.setPivot(x_c_guess,y_c_guess);

% true plant
params.l = 1; % length
params.g= (2/3)*a;
params.m= 3/b;
params.t_m = params.m * params.g * params.l; % torque limit on input
params.b = 0.0;  % damping
params.mu = 0.3; % coefficient of friction
p = PendulumPlant01(params);
p.setPivot(x_c,y_c);

% mpc parameters
mpc_params.Nmpc = 20;    % mpc horizon
mpc_params.Ntraj = 100;  % trajectory length
mpc_params.dt = 0.02;    % time-step
mpc_params.QN = blkdiag(eye(p.nq), 0.1*eye(p.nv));
mpc_params.Q = blkdiag(eye(p.nq), 0.1*eye(p.nv));
mpc_params.R = 0.001*eye(p.nu);

% kalman filter parameters
R=.1*eye(4);
Q=.1*eye(8);
P=.1*eye(8);

xk = [x_c; y_c; x; 0; 0; dxdt]; % true initial state
xk_guess = [x_c_guess; y_c_guess; x_guess; 0; 0; dxdt_guess]; % guess initial state

xg = [x_c; y_c; pi; 0; 0; 0]; % goal state
ug = [0; p_guess.m * p_guess.g;
    0.5 * p_guess.m * p_guess.g * p_guess.l*sin(pi)];

% nominal trajectory
mpc_params.x0 = xg; %repmat(xg, 1, mpc_params.Ntraj);
mpc_params.u0 = ug; %repmat(ug, 1, mpc_params.Ntraj);

% mpc_params2 = mpc_params;
% mpc_params2.Xnom = repmat(xg, 1, mpc_params.Ntraj);
% mpc_params2.Unom = repmat(ug, 1, mpc_params.Ntraj);

% build mpc
mpc_tv = TimeVaryingMPC2(p, mpc_params);
% mpc_tv2 = TimeVaryingMPC(p, mpc_params2);

% plotting
xvec = xk;
X_guessvec = X_guess;
uvec = [];
utruevec = [];
cvec = [];
Pvec = P;


% xk and xkp1 are in the format used in the mpc: [xp, yp, tht, vx_p, vy_p,
% omega];

% X and X_guess is in the format [tht_c; omega; a; b; tht_0; xp; yp; R];  

for k=1:mpc_tv.Ntraj
    disp(k)
%     disp(X_guess');
    
    % compute control input
    [dx_mpc, dU_mpc] = mpc_tv.run_mpc(xk_guess, true);
%     [dx_mpc2, dU_mpc2] = mpc_tv2.run_mpc(k, xk_guess);
    uk = mpc_tv.u0 + dU_mpc(1:mpc_tv.nu);
%     uk2 = mpc_tv2.Unom(:,k) + dU_mpc2(1:mpc_tv.nu);

%     u=.1*sin(2*k*mpc_params.dt)-.0001*X_guess(2);
%     uk = [0; 0; u]; 
    
    % advance true state
    [xkp1, uk_true] = p.dynamics_solve(xk, [0;0;uk(3)], mpc_tv.dt);
    
    % KF
    Z = p.my_KalmannOutputNoPartials(X);
    [dXdt_guess,dPdt]= p_guess.extended_kalmann_update(Z,X_guess,...
        uk(3),P,Q,R);
    
    % advance guess x
    P=P+mpc_tv.dt*dPdt;
    X_guess=X_guess+mpc_tv.dt*dXdt_guess;
    xk_guess = [X_guess(6); X_guess(7); X_guess(1) - X_guess(5);
        0; 0; X_guess(2)];
    
    % re-build mpc w/ new parameters
%     mpc_tv = TimeVaryingMPC(p_guess, mpc_params);
    
    
    % store solution
    xvec = [xvec, xkp1];
    uvec = [uvec, uk];
    utruevec = [utruevec, uk_true];
    cvec = [cvec, p.equality_const(xk, uk)];
    X_guessvec = [X_guessvec, X_guess];
    Pvec = [Pvec, P];
    
    % update true x
    xk = xkp1;
    X(1) = xkp1(3) + X(5);
    X(2) = xkp1(6);
end


% save('constrained_pend_pivot', 'xvec', 'uvec', 'utruevec', 'cvec');


%% Plotting

t = 0:(size(xvec, 2)-1);
disp(t(end) * mpc_tv.dt);
% t = (0:(mpc_tv.Ntraj));

% animation
figure(1); clf;
hold on; axis equal;
xlim(x_c + [-1.5, 1.5]*p.l)
ylim(y_c + [-1.5, 1.5]*p.l)
ph = plot( [xvec(1, 1); xvec(1, 1) + p.l * sin(xvec(3,1))], ...
    [ xvec(2, 1); xvec(2, 1) - p.l * cos(xvec(3,1))]);
th = title(sprintf('time: %f', 0.0));
for i = 1:numel(t)
    set(ph, 'Xdata',  [ xvec(1, i); xvec(1, i) + p.l * sin(xvec(3,i))], ...
        'Ydata', [ xvec(2, i); xvec(2, i) -  p.l * cos(xvec(3,i))])
    set(th, 'string', sprintf('time: %f', t(i)*mpc_tv.dt))
    pause(mpc_tv.dt)
end

% state
figure(2); clf;

titles = {'x', 'y', 'tht', 'vx', 'vy', 'omega'};
multiplier = [1, 1, (180/pi), 1, 1, (180/pi)];

for k = 1:(p.nq + p.nv)
    subplot(2, 3, k); hold on;
    plot(t, multiplier(k)*xvec(k, :));
    
    if k == 1
        shadedErrorBar(t, multiplier(k)*X_guessvec(6, :), ...
            multiplier(k)*sqrt(Pvec(6, 6:8:end)))
    end
    
    if k == 2
        shadedErrorBar(t, multiplier(k)*X_guessvec(7, :), ...
            multiplier(k)*sqrt(Pvec(7, 7:8:end)))
    end
    
    if k == 3
        shadedErrorBar(t, multiplier(k)*X_guessvec(1, :), ...
            multiplier(k)*sqrt(Pvec(1, 1:8:end)))
    end
    if k == 6
        shadedErrorBar(t, multiplier(k)*X_guessvec(2, :), ...
            multiplier(k)*sqrt(Pvec(2, 2:8:end)))
    end
    title(titles{k})
end

figure(5); clf;
subplot(3,1,1); hold on;
shadedErrorBar(t, X_guessvec(3, :), sqrt(Pvec(3, 3:8:end)))
yline(a)
title('Gravity-ish term')
subplot(3,1,2); hold on;
shadedErrorBar(t, X_guessvec(4, :), sqrt(Pvec(4, 4:8:end)))
yline(b)
title('Mass-ish term')
subplot(3,1,3); hold on;
shadedErrorBar(t, X_guessvec(5, :), sqrt(Pvec(5, 5:8:end)))
yline(theta_0)
title('Theta_0')

% input
figure(3); clf;
titles = {'fx', 'fy', 'tau'};
for k = 1:p.nq
    subplot(1, 3, k); hold on;
    p1 = plot(t(1:end-1), uvec(k, :));
    p2 = plot(t(1:end-1), utruevec(k, :), '--');
    title(titles{k})
    %     legend(p1, p2, 'Linear Estimate', 'True Force');
end

% velocity constraint
figure(4); clf;
titles = {'pivot-vx', 'pivot-vy'};
for k = 1:p.neq
    subplot(2, 1, k); hold on;
    plot(t(1:end-1), cvec(k, :));
    title(titles{k});
end