clear; clc; close all;

t = 0;
for i = 1:100
    
    M = rand(50);
    tic;
    Minv = matrix_inverse(M);
    disp(1/toc)
    t = t + toc; 
end

disp(t/100)