float32[] A_pivot_left          # A in Ax <= b for left ground friction
float32[] b_pivot_left          # b in Ax <= b for left ground friction
float32[] A_pivot_right         # A in Ax <= b for right ground friction
float32[] b_pivot_right         # b in Ax <= b for right ground friction
bool use_env_left               # Use conservative defaults for left if False
bool use_env_right              # Use conservative defaults for right if False

float32[] A_robot_left          # A in Ax <= b for left robot friction
float32 b_robot_left            # b in Ax <= b for left robot friction
float32[] A_robot_right         # A in Ax <= b for right robot friction
float32 b_robot_right           # b in Ax <= b for right robot friction
bool use_robot                  # Use conservative defaults if False
