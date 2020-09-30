% Test MPC on pendulum stbalizaton about upright
clear; clc; close all;
addpath('./Plants', './Solvers', '../ControlCode/')


% physical parameters (MORE INTUITIVE TO TUNE)
mass = 0.25; % kg
length = 0.1; % m
gravity = 10; % m/s^2
inertia = mass*length^2; 

%true parameters
x= pi/2 - pi/12;
dxdt=0;
a=mass*length*gravity;
b=1/inertia;
theta_0=0;
x_c=0;
y_c=0;
R= length;
X=[x;dxdt;a;b;theta_0;x_c;y_c;R];

% initial guess parameters
x_guess= x+0.5*(rand()-.5);
dxdt_guess=dxdt+0*(rand()-.5);
a_guess=a+0.5*a*(rand()-.5);
b_guess=b+0.5*b*(rand()-.5);
theta_0_guess=theta_0+0*(rand()-.5);
x_c_guess=x_c+length*(rand()-.5);
y_c_guess=y_c+length*(rand()-.5);
R_guess=R+length*(rand()-.5);
X_guess= [x_guess;dxdt_guess;a_guess;b_guess;theta_0_guess;x_c_guess;y_c_guess;R_guess];

% true pendulum plant parameters
params.g=gravity;                   % gravity (m/s^2)
l_cm=params.g/(a*b);
params.m=a/(params.g*l_cm);        % mass  (kg)
params.I_cm=0;                     % moment of inertia about center of mass;
params.l_contact = 0.25 * length;  % length of robot contact
params.mu_pivot=0.3;     % coefficient of friction at obj/ground contact
params.mu_contact=0.3;   % coefficient of friction at obj/robot contact
params.Nmax_pivot= 100;   % maximum force the ground can exert on obj along contact normal
params.Nmax_contact = 10;    % maximum force the robot can exert on obj along contact normal
params.contact_normal=[1;0]; % direction of the contact normal in the body frame
params.contact_point=R*[1;0];                   %location of contact point in the body frame
params.r_cm=l_cm*[cos(theta_0);sin(theta_0)];    %location of center of mass in the body frame
p = PyramidPlant01(params);
p.setPivot(x_c,y_c);

% initial guess pendulum plant
params_guess.g=gravity;            % gravity (m/s^2)
l_cm_guess=params_guess.g/(a_guess*b_guess); % length of robot contact
params_guess.m=a_guess/(params_guess.g*l_cm_guess);       % mass  (kg)
params_guess.I_cm=0; % moment of inertia about center of mass;
params_guess.l_contact = 0.25 * length;
params_guess.mu_pivot=0.3;     % coefficient of friction at obj/ground contact
params_guess.mu_contact=0.3;   % coefficient of friction at obj/robot contact
params_guess.Nmax_pivot=100;    % maximum force the ground can exert on obj along contact normal
params_guess.Nmax_contact=10;    % maximum force the robot can exert on obj along contact normal
params_guess.contact_normal=[1;0]; % direction of the contact normal in the body frame
params_guess.contact_point=R_guess*[1;0];                   %location of contact point in the body frame
params_guess.r_cm=l_cm_guess*[cos(theta_0_guess);sin(theta_0_guess)];    %location of center of mass in the body frame
p_guess = PyramidPlant01(params_guess);
p_guess.setPivot(x_c_guess,y_c_guess);


% mpc parameters
mpc_params.Nmpc = 10;    % mpc horizon
mpc_params.Ntraj = 200;  % trajectory length
mpc_params.dt = 0.01;    % time-step
mpc_params.QN = blkdiag(eye(p.nq), 0.1*eye(p.nv));
mpc_params.Q = blkdiag(eye(p.nq), 0.1*eye(p.nv));
mpc_params.R = 0.001*eye(p.nu);

% kalman filter parameters
R=.1*eye(4);
Q=.01*eye(8);
P=.1*eye(8);

xk = [x_c; y_c; x; 0; 0; dxdt]; % true initial state
xk_guess = [x_c_guess; y_c_guess; x_guess; 0; 0; dxdt_guess]; % guess initial state

xg = [x_c; y_c; pi/2; 0; 0; 0]; % goal state
ug = [0; 0; 0];  % goal input TODO: should not always be zero

% state/input to linearize about for mpc
mpc_params.x0 = xg;
mpc_params.u0 = ug;

% build mpc
mpc_tv = TimeVaryingMPC2(p_guess, mpc_params);

% plotting
xvec = xk;
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
% 
% animation
figure(1); clf;
hold on; axis equal;
xlim(x_c + 0.5 * [-1.5, 1.5]*norm(p.contact_point))
ylim(y_c + [0, 1.5]*norm(p.contact_point))
p.pendulum_rigid_body_object.initialize_visualization()


% contact point
rc_w = PolygonMath.theta_to_rotmat(xvec(3, 1))*p.contact_point + [x_c; y_c];
pcontact = plot(rc_w(1), rc_w(2), 'g.', 'MarkerSize', 10);

% force at pivot
fpivot = plot(x_c + [0, lvec(1,1)], x_c + [0, lvec(2,1)], 'k');

% force at contact
fcontact = plot(rc_w(1) + [0, uvec(1,1)], rc_w(2) + [0, uvec(2,1)], 'g');
fcontact_max = plot(rc_w(1) +  [0, length * p.mu_contact], rc_w(2) + [0, -length], 'g--');
fcontact_min = plot(rc_w(1) + [0, -length * p.mu_contact], rc_w(2) + [0, -length], 'g--');

th = title(sprintf('time: %f', 0.0));
for i = 1:numel(t)
    p.pendulum_rigid_body_object.set_state(xvec(1:2, i), 0*xvec(1:2, i), ...
        0 * xvec(1:2, i), xvec(3, i), 0, 0)
    p.pendulum_rigid_body_object.update_visualization()
  
    rc_w = PolygonMath.theta_to_rotmat(xvec(3, i))*p.contact_point + [x_c; y_c];
    set(pcontact, 'Xdata', rc_w(1), 'Ydata', rc_w(2)); 
    
    % plot inputs
    if i < numel(t)
        set(fpivot, 'Xdata', x_c + [0, lvec(1,i)], 'Ydata', y_c + [0, lvec(2,i)]);        
        plot(x_c + [0, -length * p.mu_pivot], y_c + [0, length], 'k--')
        plot(x_c + [0, length * p.mu_pivot], y_c + [0, length], 'k--')
        
        set(fcontact, 'Xdata', rc_w(1) + [0, uvec(1,i)], 'Ydata', rc_w(2) + [0, uvec(2,i)]);        
        set(fcontact_max, 'Xdata', rc_w(1) + [0, length * p.mu_contact], 'Ydata', ...
            rc_w(2) + [0, -length]);       
        set(fcontact_min, 'Xdata', rc_w(1) +  [0, -length * p.mu_contact], 'Ydata', ...
            rc_w(2) + [0, -length]);       
    
    end
    xlim(x_c + 0.5 * [-1.5, 1.5]*norm(p.contact_point))
    ylim(y_c + [0, 1.5]*norm(p.contact_point))
    pause(0.1 * mpc_tv.dt)    
end

% state estimates
figure(2); clf;

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
figure(3); clf;
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
figure(4); clf;
titles = {'f_n', 'f_t', 'tau'};
for k = 1:p.nq
    subplot(1, 3, k); hold on;
    p1 = plot(t(1:end-1), uvec(k, :));
    title(titles{k})
end

% recation forces
figure(5); clf; 
titles = {'\lambda_x', '\lambda_y'};
for k = 1:2
    subplot(1, 2, k); hold on;
    p1 = plot(t(1:end-1), lvec(k, :));
    title(titles{k})
end

% velocity constraint
figure(6); clf;
titles = {'pivot-vx', 'pivot-vy'};
subplot(2, 1, 1); hold on;
plot(t, xvec(4, :));
title(titles{1});
subplot(2, 1, 2); hold on;
plot(t, xvec(5, :));
title(titles{2});
