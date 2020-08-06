clear; close all; 

syms m l g I
syms x y tht
syms vx vy omg
syms tau

M = [m 0 m*l*cos(tht); 0 m m*l*sin(tht); m*l*cos(tht) m*l*sin(tht) 4*m*l^2/3];
c = [-m*l*omg^2*sin(tht); m*g + m*l*omg^2*cos(tht); m*g*l*sin(tht)]; 
B = [0; 0; 1];

A = [1 0 0;
    0 1 0]; 

Ad = [0 0 0;
    0 0 0]; 


q = [x; y; tht];
qd = [vx; vy; omg]; 


LHS = simplify(A*inv(M)*transpose(A));
pretty(LHS)

temp1 = simplify(Ad * qd);
temp2 = B*tau - c;
temp3 = simplify(A*inv(M));


RHS = simplify(temp3*temp2 + temp1);
pretty(RHS)
