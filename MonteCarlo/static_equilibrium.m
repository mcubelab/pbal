function [ueq, lambda, isfeasible] = static_equilibrium(x,plant)


q = x(1:plant.nq);
qd = 0 * q; 

% f([q; 0], 0) + df_du * uk = 0
[f, ~, df_du] = plant.dynamics([q; qd], zeros(plant.nu, 1));

% c([q; 0], 0) + df_du * uk <= 0
[c, ~, dc_du] = plant.inequality_const([q; qd], zeros(plant.nu, 1));

% try to solve for static equilibrium
[ueq, ~, exitflag] = linprog(zeros(plant.nu, 1), dc_du, -c, df_du, -f, ...
    [], [], [], optimoptions('linprog', 'display', 'none'));  

if exitflag == 1
    isfeasible = 1;    
    % lambda
    plant.dynamics([q; qd], ueq);
    lambda = plant.sticking_constraint_ground.getMultipliers();

elseif exitflag == -5 || exitflag == -2
    isfeasible = 0;
    ueq = zeros(plant.nu, 1);
    lambda = zeros(2, 1); 
else
    error('Static equilibrium test inconclusive')
end

end

