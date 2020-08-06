clear; clc; close all; 

syms m l g I
syms x y tht
syms vx vy omg
syms tau

M = [m 0 0; 0 m 0; 0 0 m*l^2/3];
c = [0; m*g; 0]; 
B = [0; 0; 1];

A = [1 0 -l*cos(tht);
    0 1 -l*sin(tht)]; 

Ad = [0 0 omg*l*sin(tht);
    0 0 -omg*l*cos(tht)]; 


q = [x; y; tht];
qd = [vx; vy; omg]; 



LHS = simplify(A*inv(M)*transpose(A));
pretty(LHS)

% pretty(temp3)

temp1 = simplify(Ad * qd);
temp2 = B*tau - c;
temp3 = simplify(A*inv(M));

RHS = simplify(temp3*temp2 + temp1);
pretty(RHS)
