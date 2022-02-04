# primitive info
string name                             # primitive name
int8 mode                                 # primitive mode
string error_dict_keys                    # current primitive error keys
float32[] error_dict_vals                 # current primitive error vals
bool snewrb                               # did state exist 

# QP costs
float32[] quadratic_cost_term             # Q in cost: 0.5*dw^T*Q*dw + l^T*dw
float32[] linear_cost_term                # l in cost: 0.5*dw^T*Q*dw + l^T*dw
float32[] proj_vec_list                   # motion directions in cost
float32[] error_list                      # list of errors?

# QP constraints
string label_list_cnstr                      # comma constraint names
float32[] constraint_normals              # A in A*dw <= b
float32[] constraint_offsets              # biq
float32[] measured_wrench                 # w
float32[] slacks                          # b in A*dw <= b, equal to A*w - biq

# Solution
float32 solve_time                          # time to solve [s]
float32[] delta_wrench                    # solution to QP (dw)
float32[] delta_wrench_unconstrained      # solution to QP w/o constraints

