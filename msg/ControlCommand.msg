string name                     # name of current command
bool command_flag               # 0 = absolute, 1 = relative
int8 mode                       # primitive

# relative commands
float32 delta_theta
float32 delta_s_pivot
float32 delta_s_hand

# absolute commands
float32 theta
float32 s_pivot
float32 s_hand