function ueq = find_max_torque(x, sign, plant)
%FIND_MAX_TORQUE Summary of this function goes here
%   Detailed explanation goes here


% current state
q = x(1:plant.nq);
qd = x(plant.nq + (1:plant.nv));

% c([q; 0], 0) + df_du * uk <= 0
[c, ~, dc_du] = plant.inequality_const([q; qd], zeros(plant.nu, 1));

% get location of contact point world frame
plant.pendulum_rigid_body_object.set_p_and_theta([q(1); q(2)], q(3));
cont_pt_WF = plant.pendulum_rigid_body_object.rigid_body_position ...
    (obj.contact_point);

% moment arm of torque
moment_arm = cont_pt_WF - [q(1); q(2)];


l = -sign*[-moment_arm(2), moment_arm(1), 1];

% try to solve for static equilibrium
[ueq, ~, ~] = linprog(l, dc_du, -c, [], [], ...
    [], [], [], optimoptions('linprog', 'display', 'final'));  

% if exitflag == 1
%     isfeasible = 1;    
%     % lambda
%     plant.dynamics([q; qd], ueq);
%     lambda = plant.sticking_constraint_ground.getMultipliers();
% 
% elseif exitflag == -5 || exitflag == -2
%     isfeasible = 0;
%     ueq = zeros(plant.nu, 1);
%     lambda = zeros(2, 1); 
% else
%     error('Static equilibrium test inconclusive')
% end

end

