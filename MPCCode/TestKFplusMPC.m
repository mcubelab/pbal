% Test MPC on pendulum stbalizaton about upright
clear; clc; close all;
addpath('./Plants', './Solvers', '../ControlCode/')


%true parameters
x= pi/2 - pi/6;
dxdt=0;
a=7;
b=1;
theta_0=0;
x_c=1;
y_c=-2;
R=8;
X=[x;dxdt;a;b;theta_0;x_c;y_c;R];

% inital guess
x_guess= x+0.5*(rand()-.5);
dxdt_guess=dxdt+0*(rand()-.5);
a_guess=a+2*(rand()-.5);
b_guess=b+0.3*(rand()-.5);
theta_0_guess=theta_0+0*(rand()-.5);
x_c_guess=x_c+1*(rand()-.5);
y_c_guess=y_c+1*(rand()-.5);
R_guess=R+2*(rand()-.5);
X_guess=[x_guess;dxdt_guess;a_guess;b_guess;theta_0_guess;x_c_guess;y_c_guess;R_guess];

% true pendulum plant
params.g=10;            % gravity (m/s^2)
l_cm=params.g/(a*b);

params.m=a/(params.g*l_cm);      % mass  (kg)
params.I_cm=0; % moment of inertia about center of mass;
params.t_m=100;   % control torque limit (N*m)

params.l_contact =.1;
params.mu_pivot=0.3;     % coefficient of friction at obj/ground contact
params.mu_contact=0.3;   % coefficient of friction at obj/robot contact
params.Nmax_pivot=10;   % maximum force the ground can exert on obj along contact normal
params.Nmax_pivot=10; % maximum force the robot can exert on obj along contact normal
%             obj.l_contact = params.l_contact;     % length of object/robot contact
params.contact_normal=[1;0]; % direction of the contact normal in the body frame


params.contact_point=R*[1;0];                   %location of contact point in the body frame
params.r_cm=l_cm*[cos(theta_0);sin(theta_0)];    %location of center of mass in the body frame

p = PyramidPlant01(params);
p.setPivot(x_c,y_c);

% kalman pendulum plant
params_guess.g=10;            % gravity (m/s^2)
l_cm_guess=params_guess.g/(a_guess*b_guess);

params_guess.m=a/(params_guess.g*l_cm_guess);       % mass  (kg)
params_guess.I_cm=0; % moment of inertia about center of mass;
params_guess.t_m=100;   % control torque limit (N*m)

params_guess.l_contact =.1;
params_guess.mu_pivot=0.3;     % coefficient of friction at obj/ground contact
params_guess.mu_contact=0.3;   % coefficient of friction at obj/robot contact
params_guess.Nmax_pivot=10;   % maximum force the ground can exert on obj along contact normal
params_guess.Nmax_pivot=10; % maximum force the robot can exert on obj along contact normal
%             obj.l_contact = params.l_contact;     % length of object/robot contact
params_guess.contact_normal=[1;0]; % direction of the contact normal in the body frame


params_guess.contact_point=R_guess*[1;0];                   %location of contact point in the body frame
params_guess.r_cm=l_cm_guess*[cos(theta_0_guess);sin(theta_0_guess)];    %location of center of mass in the body frame


p_guess = PyramidPlant01(params_guess);
p_guess.setPivot(x_c_guess,y_c_guess);



% mpc parameters
mpc_params.Nmpc = 10;    % mpc horizon
mpc_params.Ntraj = 500;  % trajectory length
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
% ug = [0; -p.m; 0.5 * p.m * p.g * p.l*sin(pi)];

ug = [0; 0; 0];  %TODO: should not always be zero

% goal state
mpc_params.x0 = xg;
mpc_params.u0 = ug;

% build mpc
mpc_tv = TimeVaryingMPC2(p_guess, mpc_params);

% plotting
xvec = xk;
X_guessvec = X_guess;
uvec = [];
lvec = [];
cvec = [];
Pvec = P;


% xk and xkp1 are in the format used in the mpc: [xp, yp, tht, vx_p, vy_p,
% omega];

% X and X_guess is in the format [tht_c; omega; a; b; tht_0; xp; yp; R];  

for k=1:mpc_tv.Ntraj
%     
%     if k == 1
%         update_linearization_flag = true;
%     else
%         update_linearization_flag = false;
%     end
%     
    % compute control input
    tic; 
    [dx_mpc, dU_mpc] = mpc_tv.run_mpc(xk_guess, true);
    uk = (mpc_tv.u0 + dU_mpc(1:mpc_tv.nu));
%     uk=[0;0;-3*X(1)-.3*X(2)-(X_guess(3)/X_guess(4))*sin(X_guess(5))];
    dt1 = toc; 
    
    % advance true state
    [xkp1, lk] = p.dynamics_solve(xk, uk, mpc_tv.dt);
    
    % KF
    tic; 
    Z = p.my_KalmannOutputNoPartials(X);
    [dXdt_guess,dPdt]= p_guess.extended_kalmann_update(Z,X_guess,...
        uk,P,Q,R);
%     
    % advance guess x
    P=P+mpc_tv.dt*dPdt;
    X_guess=X_guess+mpc_tv.dt*dXdt_guess;
    xk_guess = [X_guess(6); X_guess(7); X_guess(1) - X_guess(5);
        0; 0; X_guess(2)];   %TODO: NOT SURE IF THIS IS RIGHT
    dt2 = toc; 
    
    fprintf('\r MPC Rate: %f,  KF Rate: %f, Total Rate: %f', ...
        1/dt1, 1/dt2, 1/(dt1 + dt2))    
%     
%     % store solution
    xvec = [xvec, xkp1];
    uvec = [uvec, uk];
    lvec = [lvec, lk];
%     cvec = [cvec, p.equality_const(xk, uk)];
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

% animation
figure(1); clf;
hold on; axis equal;
xlim(x_c + [-1.5, 1.5]*norm(p.r_cm))
ylim(y_c + [-1.5, 1.5]*norm(p.r_cm))
p.pendulum_rigid_body_object.initialize_visualization()
% ph = plot( [xvec(1, 1); xvec(1, 1) + p.l * sin(xvec(3,1))], ...
%     [ xvec(2, 1); xvec(2, 1) - p.l * cos(xvec(3,1))]);
th = title(sprintf('time: %f', 0.0));
for i = 1:numel(t)
    p.pendulum_rigid_body_object.set_state(xvec(1:2, i), 0*xvec(1:2, i), ...
        0 * xvec(1:2, i), xvec(3, i), 0, 0)
    p.pendulum_rigid_body_object.update_visualization()
    xlim(x_c + [-1.5, 1.5]*norm(p.r_cm))
    ylim(y_c + [-1.5, 1.5]*norm(p.r_cm))
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
%     if k < p.nq
%         p2 = plot(t(1:end-1), lvec(k, :), '--');
%     end
    title(titles{k})
    %     legend(p1, p2, 'Linear Estimate', 'True Force');
end

% velocity constraint
% figure(4); clf;
% titles = {'pivot-vx', 'pivot-vy'};
% for k = 1:p.neq
%     subplot(2, 1, k); hold on;
%     plot(t(1:end-1), cvec(k, :));
%     title(titles{k});
% end