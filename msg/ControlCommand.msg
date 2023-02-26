string name                     # name of current command
int8 command_flag               # 0 = absolute, 1 = relative
int8 mode                       # primitive

# relative commands
float32 delta_theta
float32 delta_s_pivot
float32 delta_s_hand

float32 delta_theta_relative
float32 delta_theta_object

# absolute commands
float32 theta
float32 s_pivot
float32 s_hand

float32 theta_relative
float32 theta_object

#jog commands
float32 delta_x
float32 delta_y
float32 delta_normal
float32 delta_tangential