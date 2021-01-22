function res=end_effector_inertia01()

M=.0549; %mass of end effector in kg
m_old=0.28;

r_cm_onshape=[4.584 ;0.249;29.901]/1000; %center of mass location in meters

Icm_onshape=(M/m_old)*(1/10^6)*[ 182.759 ,-0.901   ,15.247;...
     -0.901   ,163.181  ,0.926;...
      15.247  , 0.926    ,79.413]; %
  
e_x_onshape_in_flange=[sqrt(2)/2;sqrt(2)/2;0];
e_y_onshape_in_flange=[-sqrt(2)/2;sqrt(2)/2;0];
e_z_onshape_in_flange=[0;0;1];
Transformation_from_onshape_to_flange=[e_x_onshape_in_flange,e_y_onshape_in_flange,e_z_onshape_in_flange];

r_cm_flange=Transformation_from_onshape_to_flange*r_cm_onshape
Icm_flange=Transformation_from_onshape_to_flange*Icm_onshape*Transformation_from_onshape_to_flange'

end