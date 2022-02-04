string name                     # name of current command
bool command_flag               # 0 = absolute, 1 = relative
int8 mode                       # primitive

# relative commands
float32 delta_theta
float32 delta_x_pivot
float32 delta_s

# absolute commands
float32 theta
float32 x_pivot
float32 s