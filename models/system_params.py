import numpy as np

class SystemParams(object):

    def __init__(self):

        self.object_params = {
            "L_CONTACT_MAX": 0.1,                     # m (length of robot/object contact)
            "MU_GROUND_0": None,                      # friction between obj/ground
            "MU_CONTACT_0": None,                     # friciton between robot/obj
            "TORQUE_BOUNDARY_MARGIN": 0.8,            # multiplies L_CONTACT to set conservative margin for torque boundary
            "END_EFFECTOR_MASS": 0.193  ,             # hand mass (kg)

        }

        self.ground_truth_params = {
            "SHAPE_NAME": "triangle-no-mass",
            "THETA_GROUND": 0.0,
            "GROUND_OFFSET": 0.0,
            "GROUND_CONTACT_MARGIN": .003,
            "ANGLE_THRESHOLD": np.pi/10,
            "COLLISION_MARGIN": .008,
            "APRIL_TAG_OFFSET": np.array([-.0055,-.003]),
            "RATE": 100
        }

        self.estimator_params = {
            "RATE": 100.,                           # hz
            "NBATCH_PIVOT": 250,                    # max number of datapoints for estimation for pivot 
            "UPDATE_LENGTH_PIVOT": 40, #200            # number of good points before update/publish for pivot
            "ANGLE_DIFF_THRESH_PIVOT": 0.005,       # rad(difference in angles for new datapoints for pivot)
            "SLIDING_THRESH_FRICTION_EST": 0.03,    # m/s (threshold for sliding velocity)
            "NBATCH_GRAV_PARAMS": 500,              # number of good points before update/publish for grav
            "ANGLE_DIFF_THRESH_GRAV": 0.001,        # rad (difference in angles for new datapoints for grav)
            "DELTA_ANGLE_THRESH_GRAV": 0.25,        # rad (minimum travel needed to publish grav params)
            "NORMAL_FORCE_THRESHOLD_FORCE": 0.05    # N (min normal force to register as being in contact)
        }

        pivot_params= { 
            "K_theta": 150.,                                         # objective function parameters: theta, line/line plus point/line
            "theta_scale": 0.3,
            "concavity_theta": 60,
            "pure_agnostic_rotation": False,
            
            "K_s": .3,  #1.                                              # objective function parameters: s, line/line plus point/line
            "s_scale": 0.0005,
            "concavity_s": 0.3,
            
            "K_x_pivot": 108.,                                             # objective function parameters: s, line/line plus point/line
            "x_pivot_scale": 0.006,
            "concavity_x_pivot": 60,
            
            "K_N": .2,                                               # objective function parameters: N, line/line plus point/line
            "N_scale": 8.,
            "concavity_N": 1.,
            "N_tar": 20.,

            "wrench_regularization_constant": 0.00001,

            "tr_friction": [0.5 , 3.0],                            # barrier function parameters for line/line plus point/line
            "friction_margin": .7, #.8
            "mu_contact": .1,
            "use_measured_mu_contact": True,

            "tr_torque": [.2 , 6.], #[.2, 4.]
            "torque_margin": 0.06,
            "l_contact_multiplier": .95, #.9
            
            "tr_friction_external": [.9 , 1],
            "friction_ground_margin": .6, #.4, #.8
            "mu_ground": .3, # .2,
            "use_measured_mu_ground": True,

            "tr_max_normal_contact": [.3, 3.],
            "Nmax_contact": 20., #35

            "tr_min_normal_external": [1., 1]                   
        }

        guarded_move_params = {
            "K_theta": 150.,                                            # objective function parameters: theta, free
            "theta_scale": 0.3,
            "concavity_theta": 60.,
            "pure_agnostic_rotation": True, 

            "K_pos_hand": 100.,                                            # objective function parameters: x-hand, free
            "hand_pos_scale": 0.006,
            "concavity_pos_hand": 60.,

            "wrench_regularization_constant": 0.00001,

            "tr_friction": [20., 1.],        # barrier function parameters for line/line plus point/line
            "friction_margin": -2,
            "mu_contact": 0.0,
            "use_measured_mu_contact": False,

            "tr_torque": [5., 5.],
            "torque_margin": -.1,
            "l_contact_multiplier": 0.0,

            "tr_max_normal_contact": [20., 1.],
            "Nmax_contact": 2,

            "tr_min_normal_contact": [20., 1.],
            "Nmin_contact": -2
        }

        static_object_flush_move_params = {
            "K_pos_hand": 100.,                                            # objective function parameters: x-hand, free
            "hand_pos_scale": 0.006,
            "concavity_pos_hand": 60.,

            "wrench_regularization_constant": 0.01,

            "tr_friction": [1.5, .5],        # barrier function parameters for line/line plus point/line
            "friction_margin": -.7,
            "mu_contact": 0.0,
            "use_measured_mu_contact": False,

            "tr_torque": [6., 6.],
            "torque_margin": -.01,
            "l_contact_multiplier": 0.0,

            "tr_max_normal_contact": [1.5, .5],
            "Nmax_contact": .7,

            "tr_min_normal_contact": [1.5, .5],
            "Nmin_contact": -.7
        }

        static_object_flush_stick_params = {
            "wrench_regularization_constant": 0.01,

            "tr_friction": [0.5 , 3.],    # barrier function parameters for line/line plus point/line
            "friction_margin": 0.,
            "mu_contact": .1,
            "use_measured_mu_contact": False,

            "tr_torque": [6 , 6],
            "torque_margin": 0.01,
            "l_contact_multiplier": .8,

            "tr_max_normal_contact": [1.5, .5],
            "Nmax_contact": .4,
        }

        #static_object_flush_move_params = {
        #    "K_pos_hand": 100.,                                            # objective function parameters: x-hand, free
        #    "hand_pos_scale": 0.006,
        #    "concavity_pos_hand": 60.,

        #    "wrench_regularization_constant": 0.00001,

        #    "tr_friction": [1., 1.],        # barrier function parameters for line/line plus point/line
        #    "friction_margin": -.5,
        #    "mu_contact": 0.05,
        #    "use_measured_mu_contact": False,

        #    "tr_torque": [5., 5.],
        #    "torque_margin": -.02,
        #    "l_contact_multiplier": 0.1,

        #    "tr_max_normal_contact": [20., 1.],
        #    "Nmax_contact": .5
        #}

        pure_stick_params = {
            "K_N": .2,                                               # objective function parameters: N, line/line plus point/line
            "N_scale": 8.,
            "concavity_N": 1.,
            "N_tar": 15.,

            "wrench_regularization_constant": 0.00001,

            "tr_friction": [.5, 3.],        # barrier function parameters for line/line plus point/line
            "friction_margin": 0.,
            "mu_contact": 0.1,
            "use_measured_mu_contact": True,

            "tr_torque": [.2, 5.],
            "torque_margin": 0.05,
            "l_contact_multiplier": .2,

            "tr_max_normal_contact": [1., 3.],
            "Nmax_contact": 30.
        }

        slide_external_line_contact_params = {    

            # "K_theta": 150.,                                         # objective function parameters: theta, line/line plus point/line
            # "theta_scale": 0.3,
            # "concavity_theta": 60,
            # "pure_agnostic_rotation": True,        
            
            "K_x_pivot": 108.,                                             # objective function parameters: s, line/line plus point/line
            "x_pivot_scale": 0.006,
            "concavity_x_pivot": 60,
            "mu_ground": 0.2,
            "use_measured_mu_ground": True,

            "K_s": .5,                                               # objective function parameters: s, line/line plus point/line
            "s_scale": 0.0005,
            "concavity_s": 0.3,
            
            "K_N": .2,                                               # objective function parameters: N, line/line plus point/line
            "N_scale": 80.,
            "concavity_N": 1.,
            "N_tar": 0.,

            "K_tau": .2,                                               # objective function parameters: N, line/line plus point/line
            "tau_scale": 80.,
            "concavity_tau": 1.,
            "tau_tar": 0.,

            "tr_friction_external": [1 , 1],
            "friction_ground_margin": -3,
            "mu_ground": 0.0,
            "use_measured_mu_ground": False,

            "wrench_regularization_constant": 0.00001,   # was 0.01

            "tr_friction": [1, 1],        # barrier function parameters for line/line plus point/line
            "friction_margin": -2,
            "mu_contact": 0.0,
            "use_measured_mu_contact": False,

            "tr_torque": [1 , 1],
            "torque_margin": 0.00,
            "l_contact_multiplier": .8,
            
            "tr_max_normal_contact": [1, 1],
            "Nmax_contact": 30.,  

            "tr_min_normal_contact": [1, 1],
            "Nmin_contact": 1.5
        }

        self.controller_params = {
            "IMPEDANCE_STIFFNESS_LIST": [4000, 4000, 4000, 400, 120, 400],
            # "IMPEDANCE_STIFFNESS_LIST": [1000, 1000, 1000, 100, 30, 100],
            # "IMPEDANCE_STIFFNESS_LIST": [1000./5., 1000/5., 1000/5., 100/5., 30/5., 100/5.],
            "TORQUE_UPPER": [40, 40, 36, 36, 32, 28, 24],                  # default [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
            "FORCE_UPPER": [100, 100, 100, 25, 25, 25],                    # default [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
            "RATE": 100,                                                    # hz (control rate)
            "INTEGRAL_MULTIPLIER": 30., #20
            # "INTEGRAL_MULTIPLIER": 10.,
            "pivot_params": pivot_params,
            "guarded_move_params": guarded_move_params,
            "static_object_flush_move_params": static_object_flush_move_params,
            "pure_stick_params": pure_stick_params,
            "slide_external_line_contact_params": slide_external_line_contact_params,
            "static_object_flush_stick_params": static_object_flush_stick_params
        }
