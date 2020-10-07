% Test MPC on pendulum stbalizaton about upright
clear; clc; close all;
addpath('./Plants', './Solvers', '../ControlCode/')

% physical parameters (MORE INTUITIVE TO TUNE)
mass = 0.5; % kg
length = 0.1; % m
gravity = 10; % m/s^2
inertia = mass*length^2;

% fixed parameters
mu_contact = 0.5;
mu_pivot = 0.5;
Nmax_pivot = 100;
Nmax_contact = 5;
l_contact = 0.25 * length;
contact_normal = [-1; 0];

%true parameters
x= pi/2 - pi/6;
dxdt=0;
a=mass*length*gravity;
b=1/inertia;
theta_0=pi/12;
x_c=0;
y_c=0;
R= 1.5 * length;
X=[x;dxdt;a;b;theta_0;x_c;y_c;R];

% initial guess parameters
x_guess= x + (pi/12)*(rand()-.5);
dxdt_guess = dxdt + (pi/12)*(rand()-.5);
a_guess= a + 0.2 * a *(rand()-.5);
b_guess= b + 0.2 * b * (rand()-.5);
theta_0_guess=theta_0 + 0.1 * theta_0*(rand()-.5);
x_c_guess=x_c+ 0.2 * length*(rand()-.5);
y_c_guess=y_c+ 0.2 * length*(rand()-.5);
R_guess=R+ 0.2 * length*(rand()-.5);
% X_guess = X;
X_guess= [x_guess;dxdt_guess;a_guess;b_guess;theta_0_guess;x_c_guess;y_c_guess;R_guess];

% true pendulum plant parameters
params.g=gravity;                   % gravity (m/s^2)
l_cm=params.g/(a*b);
params.m=a/(params.g*l_cm);        % mass  (kg)
params.I_cm=0;                     % moment of inertia about center of mass;
params.l_contact = l_contact;  % length of robot contact
params.mu_pivot= mu_pivot;     % coefficient of friction at obj/ground contact
params.mu_contact = mu_contact;   % coefficient of friction at obj/robot contact
params.Nmax_pivot = Nmax_pivot;   % maximum force the ground can exert on obj along contact normal
params.Nmax_contact = Nmax_contact;    % maximum force the robot can exert on obj along contact normal
params.contact_normal = contact_normal; % direction of the contact normal in the body frame
params.contact_point = R*[1;0];                   %location of contact point in the body frame
params.r_cm=l_cm*[cos(theta_0);sin(theta_0)];    %location of center of mass in the body frame
p = PyramidPlant01(params);
p.setPivot(x_c,y_c);

% initial guess pendulum plant
params_guess.g=gravity;            % gravity (m/s^2)
l_cm_guess=params_guess.g/(a_guess*b_guess); % length of robot contact
params_guess.m=a_guess/(params_guess.g*l_cm_guess);       % mass  (kg)
params_guess.I_cm=0; % moment of inertia about center of mass;
params_guess.l_contact = l_contact;
params_guess.mu_pivot=mu_pivot;     % coefficient of friction at obj/ground contact
params_guess.mu_contact=mu_contact;   % coefficient of friction at obj/robot contact
params_guess.Nmax_pivot=Nmax_pivot;    % maximum force the ground can exert on obj along contact normal
params_guess.Nmax_contact=Nmax_contact;    % maximum force the robot can exert on obj along contact normal
params_guess.contact_normal=contact_normal; % direction of the contact normal in the body frame
params_guess.contact_point=R_guess*[1;0];                   %location of contact point in the body frame
params_guess.r_cm=l_cm_guess*[cos(theta_0_guess);sin(theta_0_guess)];    %location of center of mass in the body frame
p_guess = PyramidPlant01(params_guess);
p_guess.setPivot(x_c_guess,y_c_guess);


% mpc parameters
mpc_params.Nmpc = 20;    % mpc horizon
mpc_params.Ntraj = 500;  % trajectory length
mpc_params.dt = 0.01;    % time-step
mpc_params.QN = blkdiag(eye(p.nq), 0.01*eye(p.nv));
mpc_params.Q = blkdiag(eye(p.nq), 0.01*eye(p.nv));
mpc_params.R = 0.0001*eye(p.nu);

% kalman filter parameters
R=.1*eye(4);
Q=.01*eye(8);
P=.1*eye(8);

xk = [x_c; y_c; x; 0; 0; dxdt]; % true initial state
xk_guess = [x_c_guess; y_c_guess; x_guess; 0; 0; dxdt_guess]; % guess initial state

xg = [x_c; y_c; pi/2 - theta_0; 0; 0; 0]; % goal state (TODO: USING THETA_0 IS CHEATING!!!)
ug = [0; 0; 0];  % goal input TODO: should not always be zero

% state/input to linearize about for mpc
mpc_params.x0 = xg;
mpc_params.u0 = ug;

% build mpc
mpc_tv = TimeVaryingMPC2(p_guess, mpc_params);

% plotting
xvec = xk;
xvec_guess = xk_guess;
X_guessvec = X_guess;
uvec = [];
lvec = [];
Pvec = P;


% xk and xkp1 are in the format used in the mpc: [xp, yp, tht, vx_p, vy_p,
% omega];

% X and X_guess is in the format [tht_c; omega; a; b; tht_0; xp; yp; R];

for k=1:mpc_tv.Ntraj
    
    % compute control input
    tic;
    [dx_mpc, dU_mpc] = mpc_tv.run_mpc(xk_guess, true);
    uk = (mpc_tv.u0 + dU_mpc(1:mpc_tv.nu));
    dt1 = toc;
    
    % advance true state
    [xkp1, lk] = p.dynamics_solve(xk, uk, mpc_tv.dt);
    
    %
    tic;
    Z = p.my_KalmannOutputNoPartials(X);  % observation
    
    % kalmann update
    [dXdt_guess,dPdt]= p_guess.extended_kalmann_update(Z,X_guess,...
        uk,P,Q,R);
    
    % advance guess x
    P=P+mpc_tv.dt*dPdt;
    X_guess=X_guess+mpc_tv.dt*dXdt_guess;
    xk_guess = [X_guess(6); X_guess(7); X_guess(1); 0; 0; X_guess(2)];
    dt2 = toc;
    
    % update system parameters
    p_guess.UpdateParams_kalmann(X_guess);
    
    fprintf('\r MPC Rate: %f,  KF Rate: %f, Total Rate: %f', ...
        1/dt1, 1/dt2, 1/(dt1 + dt2))
    
    % store solution
    xvec = [xvec, xkp1];
    xvec_guess = [xvec_guess, xk_guess]; 
    uvec = [uvec, uk];
    lvec = [lvec, lk];
    X_guessvec = [X_guessvec, X_guess];
    Pvec = [Pvec, P];
    
    % update true x
    xk = xkp1;
    X(1) = xkp1(3) + X(5);
    X(2) = xkp1(6);
end


%% Plotting

t = 0:(size(xvec, 2)-1);

% animation
% f1 = figure(1); clf; hold on;
% s1 = get(f1, 'currentaxes');
% p.initialize_visualization(xvec(:, 1), uvec(:, 1), length/2)
% t1 = title(sprintf('time: %f', 0.0));
% axis equal; 
% xlims1 = get(s1, 'xlim');
% ylims1 = get(s1, 'ylim');

f2 = figure(2); clf; hold on;
s2 = get(f2, 'currentaxes');
p_guess.initialize_visualization(xvec_guess(:, 1), uvec(:, 1), length/2)
t2 = title(sprintf('time: %f', 0.0));
axis equal; 
xlims2 = get(s2, 'xlim');
ylims2 = get(s2, 'ylim');


nskip = 5;
for i = 1:nskip:(numel(t)-1)
      
%     f1; 
%     p.update_visualization(xvec(:, i), uvec(:, i), length/2)
%     xlim(1.25 * xlims1)
%     ylim(1.25 * ylims1)
%     title(sprintf('time: %f', mpc_tv.dt * t(i)));
%     print('-dpng', sprintf('./Animations/png/true_%3f.png',  mpc_tv.dt * t(i)))

% 
    f2; 
    p_guess.update_visualization(xvec_guess(:, i), uvec(:, i), length/2)
    xlim(1.25 * xlims2)
    ylim(1.25 * ylims2)
    title(sprintf('time: %f', mpc_tv.dt * t(i)));
    print('-dpng', sprintf('./Animations/png/guess_%3f.png',  mpc_tv.dt * t(i)))

end

% state estimates
figure(3); clf;

titles = {'x', 'y', 'tht', 'vx', 'vy', 'omega'};
multiplier = [1, 1, (180/pi), 1, 1, (180/pi)];

for k = 1:(p.nq + p.nv)
    
    subplot(2, 3, k); hold on;
    
    % guess state + uncertainty
    if k == 1  % theta
        shadedErrorBar(t, multiplier(k)*X_guessvec(6, :), ...
            multiplier(k)*sqrt(Pvec(6, 6:8:end)))
    end
    
    if k == 2  % x_c
        shadedErrorBar(t, multiplier(k)*X_guessvec(7, :), ...
            multiplier(k)*sqrt(Pvec(7, 7:8:end)))
    end
    
    if k == 3 % y_c
        shadedErrorBar(t, multiplier(k)*X_guessvec(1, :), ...
            multiplier(k)*sqrt(Pvec(1, 1:8:end)))
    end
    if k == 6 % omega
        shadedErrorBar(t, multiplier(k)*X_guessvec(2, :), ...
            multiplier(k)*sqrt(Pvec(2, 2:8:end)))
    end
    
    % true state
    plot(t, multiplier(k)*xvec(k, :), 'r');
    
    title(titles{k})
end

% parameter estimates
figure(4); clf;
subplot(3,1,1); hold on;
shadedErrorBar(t, X_guessvec(3, :), sqrt(Pvec(3, 3:8:end)))
yline(a, 'r')
title('Gravity-ish term')
subplot(3,1,2); hold on;
shadedErrorBar(t, X_guessvec(4, :), sqrt(Pvec(4, 4:8:end)))
yline(b, 'r')
title('Mass-ish term')
subplot(3,1,3); hold on;
shadedErrorBar(t, X_guessvec(5, :), sqrt(Pvec(5, 5:8:end)))
yline(theta_0, 'r')
title('Theta_0')

% input forces
figure(5); clf;
titles = {'f_x', 'f_y', 'tau'};
for k = 1:p.nq
    subplot(1, 3, k); hold on;
    p1 = plot(t(1:end-1), uvec(k, :));
    title(titles{k})
end

% recation forces
figure(6); clf;
titles = {'\lambda_x', '\lambda_y'};
for k = 1:2
    subplot(1, 2, k); hold on;
    p1 = plot(t(1:end-1), lvec(k, :));
    title(titles{k})
end

% velocity constraint
figure(7); clf;
titles = {'pivot-vx', 'pivot-vy'};
subplot(2, 1, 1); hold on;
plot(t, xvec(4, :));
title(titles{1});
subplot(2, 1, 2); hold on;
plot(t, xvec(5, :));
title(titles{2});

tilefigs; 
