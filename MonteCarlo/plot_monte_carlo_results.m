clear; clc; close all; 

filename = 'vary_length_Jan_06_2021_14_19.mat';
load(filename)

Nmc = length(monte_carlo_data);

vary_param_vec = zeros(Nmc, 1);
for i = 1:Nmc
    vary_param_vec(i) = monte_carlo_data(i).x0.R;
end

figure(1); clf; hold on; 
plot(vary_param_vec, [monte_carlo_data.is_feasible], 'ko', 'markersize', 10, ...
    'markerfacecolor', 'k')
plot(vary_param_vec, [monte_carlo_data.succeed], 'go', 'markerfacecolor', 'g')

figure(2); clf; hold on; 
for i = 1:Nmc
    p1 = plot(monte_carlo_data(i).solution.t, ...
        monte_carlo_data(i).solution.X_guessvec(8, :));
    p2 = plot(monte_carlo_data(i).solution.t(end), vary_param_vec(i), ...
        'o', 'markerfacecolor',  get(p1, 'color'), 'markeredgecolor', ...
         get(p1, 'color')); 
end

figure(3); clf; hold on; 
for i = 1:Nmc
    p1 = plot(monte_carlo_data(i).solution.t, ...
        monte_carlo_data(i).solution.X_guessvec(1, :), '--');
    plot(monte_carlo_data(i).solution.t, ...
        monte_carlo_data(i).solution.xvec(3, :), 'color', get(p1, 'color'));
end
