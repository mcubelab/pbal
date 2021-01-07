clear; clc; close all;

% filename = 'vary_length_Jan_06_2021_15_39.mat';
% filename = 'vary_length_Jan_06_2021_16_34.mat';
% filename = 'vary_theta0_Jan_06_2021_14_25.mat';
filename = 'vary_x_Jan_06_2021_16_55';
% filename = 'vary_xc_Jan_06_2021_12_15.mat';
% filename = 'vary_yc_Jan_06_2021_12_32.mat';



load(filename)

Nmc = length(monte_carlo_data);

vary_param_vec = zeros(Nmc, 1);
for i = 1:Nmc
    
%     vary_param_vec(i) = monte_carlo_data(i).x0.R;
    %     vary_param_vec(i) = 1/(monte_carlo_data(i).x0.b*(monte_carlo_data(i).x0.R)^2);
    %     vary_param_vec(i) = monte_carlo_data(i).x0.theta_0;
        vary_param_vec(i) = monte_carlo_data(i).x0.x;
    %     vary_param_vec(i) = monte_carlo_data(i).x0.x_c;
    %     vary_param_vec(i) = monte_carlo_data(i).x0.y_c;
    
end

% for i = 1:Nmc
%     contact_y(i) = monte_carlo_data(i).solution.Zvec
%     contact_y_guess(i) = monte_carlo_data(i)
%     
% end

color_range = jet(Nmc);

figure(1); clf; hold on;
plot(vary_param_vec, [monte_carlo_data.is_feasible], 'ko', 'markersize', 10, ...
    'markerfacecolor', 'k')
plot(vary_param_vec, [monte_carlo_data.succeed], 'go', 'markerfacecolor', 'g')

figure(2); clf;
subplot(3,1,1); hold on;
for i = 1:Nmc
    p1 = plot(monte_carlo_data(i).solution.t, monte_carlo_data(i).solution.X_guessvec(8, :) ...
        + monte_carlo_data(i).solution.X_guessvec(7, :), ...
        'color',color_range(i,:));
    %     1./(monte_carlo_data(i).solution.X_guessvec(4, :).*monte_carlo_data(i).solution.X_guessvec(8, :).^2)
    p2 = plot(monte_carlo_data(i).solution.t(end), vary_param_vec(i), ...
        'o', 'markerfacecolor',  color_range(i,:), 'markeredgecolor', ...
        color_range(i,:));
end
    ylabel('y_c + R')

subplot(3,1,2); hold on;
for i = 1:Nmc
    p1 = plot(monte_carlo_data(i).solution.t, monte_carlo_data(i).solution.X_guessvec(8, :), ...
        'color',color_range(i,:));
end
    ylabel('R')

subplot(3,1,3); hold on;
for i = 1:Nmc
    p1 = plot(monte_carlo_data(i).solution.t, monte_carlo_data(i).solution.X_guessvec(7, :), ...
        'color',color_range(i,:));
end
    ylabel('y_c')

figure(3);
subplot(2,1,1)
hold on
for i = 1:Nmc
    %
    %     p1 = plot(monte_carlo_data(i).solution.t, ...
    %         monte_carlo_data(i).solution.X_guessvec(1, :), '--');
    plot(monte_carlo_data(i).solution.t, ...
        monte_carlo_data(i).solution.xvec(3, :), 'color', color_range(i,:));
end
ylabel('True Angle')

subplot(2,1,2)
hold on
for i = 1:Nmc
    %
    p1 = plot(monte_carlo_data(i).solution.t, ...
        monte_carlo_data(i).solution.X_guessvec(1, :), 'color', color_range(i,:));
    %     plot(monte_carlo_data(i).solution.t, ...
    %         monte_carlo_data(i).solution.xvec(3, :), 'color', color_range(i,:));
end
ylabel('Estimated Angle')

figure(4); 
subplot(2,1,1)
hold on
for i = 1:Nmc
    plot(monte_carlo_data(i).solution.t(2:end), ...
        monte_carlo_data(i).solution.Zvec(2, :), 'color', color_range(i,:));
end
ylabel('Contact Location')

subplot(2,1,2)
hold on
for i = 1:Nmc
    %
    p1 = plot(monte_carlo_data(i).solution.t(2:end), ...
        monte_carlo_data(i).solution.Z_guessvec(2, :), 'color', color_range(i,:));
end
ylabel('Estimated Contact Location')
