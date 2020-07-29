clear; clc; close all; 


constrained_pend = load('constrained_pend.mat');
constrained_pend_pivot = load('constrained_pend_pivot.mat'); 

% state
figure(1); clf;

titles = {'x', 'y', 'tht', 'vx', 'vy', 'omega'};
multiplier = [1, 1, (180/pi), 1, 1, (180/pi)];

for k = 1:6
    subplot(2, 3, k); hold on;
    plot(multiplier(k)*constrained_pend.xvec(k, :));
    plot(multiplier(k)*constrained_pend_pivot.xvec(k, :), '--');
    title(titles{k})
end

% input
figure(2); clf;
titles = {'fx', 'fy', 'tau'};
for k = 1:3
    subplot(1, 3, k); hold on;
    plot(constrained_pend.uvec(k, :));
    plot(constrained_pend_pivot.uvec(k, :), '--');
    title(titles{k})
end

% input
figure(3); clf;
titles = {'fx true', 'fy true', 'tau true'};
for k = 1:3
    subplot(1, 3, k); hold on;
    plot(constrained_pend.utruevec(k, :));
    plot(constrained_pend_pivot.utruevec(k, :), '--');
    title(titles{k})
end

% velocity constraint
figure(4); clf;
titles = {'pivot-vx', 'pivot-vy'};
for k = 1:2
    subplot(2, 1, k); hold on;
    plot(constrained_pend.cvec(k, :));
    plot(constrained_pend_pivot.cvec(k, :), '--');
    title(titles{k});
end