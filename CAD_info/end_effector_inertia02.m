function res=end_effector_inertia01()

M=.478; %mass in kg
R=40*10^(-3); %radius in meters
H=140*10^(-3); %height in meters

center_of_mass_offset = H/3
Iz= (3/10)*M*R^2
Ix= M*((3/20)*R^2+(3/80)*H^2)
end