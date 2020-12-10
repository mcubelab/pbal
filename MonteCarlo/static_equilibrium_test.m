% Test MPC on pendulum stbalizaton about upright
clear; clc;
addpath('./Plants', './Solvers', '../ControlCode/')

% physical parameters (MORE INTUITIVE TO TUNE)
mass = 1.0; % kg
length = 0.1; % m
gravity = 10; % m/s^2
inertia = mass*length^2;

% fixed parameters
mu_contact = 0.3;
mu_pivot = 0.3;
Nmax_pivot = 100;
Nmax_contact = 5;
l_contact = 0.0 * length;
tht_normal = 0;
contact_normal = [-cos(tht_normal); sin(tht_normal)];

%true parameters
a=mass*length*gravity;
b=1/inertia;
theta_0=0; %pi/12;
x_c=0;
y_c=0;
R= 1.5 * length;

% true pendulum plant parameters
params.g=gravity;                   % gravity (m/s^2)
l_cm=params.g/(a*b);
params.m=a/(params.g*l_cm);        % mass  (kg)
params.I_cm = 0; %0.1 * params.m * length^2;                     % moment of inertia about center of mass;
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


N = 100;
theta = linspace(0, pi, N);
% theta = 0;
lambda = zeros(2, N);
ueq = zeros(2, N);

figure(1); clf; hold on;
xlim([-0.2, 0.2])
ylim([-0.2, 0.2])
for i = 1:N
    xk = [0; 0; theta(i); 0; 0; 0]; 
    [ueq, lambda(:, i), exitflag(i)] = static_equilibrium(xk, p);
    if i == 1
        p.initialize_visualization(xk, ueq, length)
    else
        p.update_visualization(xk, ueq, length)
    end
    if exitflag(i) == 0
        title('Infeasible')
        disp('Infeasible')
    else
        title('Feasible')
        disp('Feasible')
        disp(ueq)
%         f = p.dynamics(xk, ueq);
        [c, dc_dx, dc_du] = p.inequality_const(xk, ueq);
        disp(c)
    end
    drawnow; 
%     pause(0.1)
end


%%
figure(2); hold on; 
plot(theta, exitflag, 'o-', 'MarkerSize', 4); 

figure(3); 
subplot(1,2,1); hold on;
plot(theta, lambda(1,:), '-', 'MarkerSize', 4); 
subplot(1,2,2); hold on;
plot(theta, lambda(2,:), '-', 'MarkerSize', 4); 