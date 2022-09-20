import numpy as np

class SystemParams(object):

    def __init__(self):

        self.object_params = {
            "L_CONTACT_MAX": .1, #.08, #0.1,                # m (length of robot/object contact)
            "MU_GROUND_0": None,                      # friction between obj/ground
            "MU_CONTACT_0": None,                     # friciton between robot/obj
            "TORQUE_BOUNDARY_MARGIN": 0.8,            # multiplies L_CONTACT to set conservative margin for torque boundary
            "END_EFFECTOR_MASS": 0.193  ,             # hand mass (kg)
        }

        self.ground_truth_params = {
            "SHAPE_NAME": "hexagon-no-mass",
            "THETA_GROUND": 0.0,
            "GROUND_OFFSET": 0.0,
            "GROUND_CONTACT_MARGIN": .003,
            "ANGLE_THRESHOLD": np.pi/10,
            "COLLISION_MARGIN": .008,
            "APRIL_TAG_OFFSET": np.array([-.0055,-.003]),
            "RATE": 100
        }

        self.estimator_params = {
            "RATE": 500.,                           # hz
            "WRENCH_CONE_EST_RATE": 100.,           # rate at which we want to update wrench cone estimates
            "NBATCH_PIVOT": 250,                    # max number of datapoints for estimation for pivot 
            "UPDATE_LENGTH_PIVOT": 40, #200         # number of good points before update/publish for pivot
            "ANGLE_DIFF_THRESH_PIVOT": 0.005,       # rad(difference in angles for new datapoints for pivot)
            "SLIDING_THRESH_FRICTION_EST": 0.03,    # m/s (threshold for sliding velocity)
            "NBATCH_GRAV_PARAMS": 500,              # number of good points before update/publish for grav
            "ANGLE_DIFF_THRESH_GRAV": 0.001,        # rad (difference in angles for new datapoints for grav)
            "DELTA_ANGLE_THRESH_GRAV": 0.25,        # rad (minimum travel needed to publish grav params)
            "NORMAL_FORCE_THRESHOLD_FORCE": 0.05    # N (min normal force to register as being in contact)
        }

        self.hal_params = {
            "RATE": 500,            # hz
            "CAMERA_RATE": 30       # hz
        }

        self.debug_params = {
            "LOG_TIME": 10,                          # time between when log messages are printed in seconds
            "QUEUE_LEN": 100                         # length of queue for computing average timing
        }

        self.pivot_params= { 

            "DEFAULT_PIVOT_LOCATION": .1,

            "K_theta": 600., #150.,                                         # objective function parameters: theta, line/line plus point/line
            "theta_scale": 0.3,
            "concavity_theta": 60,
            "pure_agnostic_rotation": False,
            
            "K_s": 1.0, #6.0, #.3,  #1.                                              # objective function parameters: s, line/line plus point/line
            "s_scale": 0.0005,
            "concavity_s": 0.3,
            
            "K_x_pivot": 800., #300., #108.,                                             # objective function parameters: s, line/line plus point/line
            "x_pivot_scale": 0.006,
            "concavity_x_pivot": 60,
            
            "K_N": .2,                                               # objective function parameters: N, line/line plus point/line
            "N_scale": 8.,
            "concavity_N": 1.,
            "N_tar": 20.,

            "wrench_regularization_constant": 0.00001,

            "tr_friction": [0.5 , 3.0], #[0.5 , 3.0],                            # barrier function parameters for line/line plus point/line
            "friction_margin": 2.0, #.9, #.8

            "mu_contact": .1,
            "use_measured_mu_contact": True,

            "tr_torque": [.8 , 6.], #[.8 , 6.], #[.2 , 6.], #[.2, 4.]
            "torque_margin": 0.03,
            "l_contact_multiplier": .95, #.9
            
            "tr_friction_external": [.9 , 1],
            "friction_ground_margin": 1.5, #.6, #.4, #.8
            "mu_ground": .3, # .2,
            "use_measured_mu_ground": True,

            "tr_max_normal_contact": [.3, 3.],
            "Nmax_contact": 32.,#32., #20., #35

            "tr_min_normal_external": [1., 1.],  

            "tr_min_normal_contact": [1., 1.], 
            "Nmin_contact": 1.,
        }


        self.controller_params = {
            # "IMPEDANCE_STIFFNESS_LIST": [4000, 4000, 4000, 400, 120, 400],
            "IMPEDANCE_STIFFNESS_LIST": [2000, 2000, 2000, 400, 60, 400],
            # "IMPEDANCE_STIFFNESS_LIST": [1000, 1000, 1000, 100, 30, 100],
            # "IMPEDANCE_STIFFNESS_LIST": [3000, 3000, 3000, 300, 90, 300],
            # "IMPEDANCE_STIFFNESS_LIST": [3000, 3000, 3000, 300, 90, 300],
            # "IMPEDANCE_STIFFNESS_LIST": [1000./5., 1000/5., 1000/5., 100/5., 30/5., 100/5.],
            "TORQUE_UPPER": [40, 40, 36, 36, 32, 28, 24],                  # default [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
            "FORCE_UPPER": [100, 100, 100, 25, 25, 25],                    # default [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
            "RATE": 200,                                                    # hz (control rate)
            # "INTEGRAL_MULTIPLIER": 20., #30., #20
            "INTEGRAL_MULTIPLIER": 25.,
        }
