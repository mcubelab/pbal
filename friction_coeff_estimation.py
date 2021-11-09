#!/usr/bin/env python

import numpy as np
import tf
import tf.transformations as tfm
import rospy
import pdb

import ros_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray, Float32, Bool
from models.system_params import SystemParams
import matplotlib.pyplot as plt

def generalized_velocities_callback(data):
    global generalized_velocities
    generalized_velocities = data

def end_effector_wrench_in_end_effector_frame_callback(data):
    global end_effector_wrench_in_end_effector_frame
    end_effector_wrench_in_end_effector_frame = data

def get_xy_wrench(wrench_list):
    return [wrench_list[0], wrench_list[1], wrench_list[-1]]

def torque_cone_boundary_test_callback(data):
    global torque_boundary_boolean
    torque_boundary_boolean = data.data


if __name__ == '__main__':

    rospy.init_node("friction_coeff_estimator")
    arm = ArmInterface()
    rospy.sleep(0.5)


    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.estimator_params["RATE"])

    # initialize globals
    end_effector_wrench_in_end_effector_frame, generalized_velocities, \
        torque_boundary_boolean = None, None, None

    #setting up subscribers
    end_effector_wrench_sub = rospy.Subscriber("/end_effector_sensor_in_end_effector_frame", 
        WrenchStamped,  end_effector_wrench_in_end_effector_frame_callback)
    generalized_velocities_sub = rospy.Subscriber("/generalized_velocities", 
        Float32MultiArray,  generalized_velocities_callback)
    torque_cone_boundary_test_sub = rospy.Subscriber("/torque_cone_boundary_test", 
        Bool,  torque_cone_boundary_test_callback)

    # setting up publisher
    robot_friction_estimate_pub = rospy.Publisher('/robot_friction_estimate', Float32, 
        queue_size=10)

    # initialize
    generalized_velocities_list = []
    end_effector_2D_wrench_list = []
    friction_list = []
    friction_estimate_message = Float32()

    # hyperparameters
    SLIDING_THRESH = sys_params.estimator_params["SLIDING_THRESH_FRICTION_EST"] # in yaml
    NOMINAL_FRICTION_VAL = sys_params.object_params["MU_CONTACT_0"]    # in yaml

    # initialize estimate
    friction_estimate = 0.
    num_measurements = 0.

    print("starting force collection")
    while not rospy.is_shutdown():

        if (end_effector_wrench_in_end_effector_frame is not None) and (
            generalized_velocities is not None) and (torque_boundary_boolean is not None):

            # end effector 2D wrench
            end_effector_2D_wrench = get_xy_wrench(ros_helper.wrench_stamped2list(
                end_effector_wrench_in_end_effector_frame))

            # if we are sliding
            if (np.abs(generalized_velocities.data[1]) > SLIDING_THRESH
                ) and torque_boundary_boolean:
                
                # if is new a velocity 
                if len(generalized_velocities_list)==0 or \
                    generalized_velocities_list[-1][1]!= generalized_velocities.data[1]:
                    
                    num_measurements+=1. # update counter

                    # frition measurement
                    friction_measurement=np.abs(end_effector_2D_wrench[1]
                        )/np.abs(end_effector_2D_wrench[0])
                    friction_list.append(friction_measurement)

                    # update estimate
                    friction_estimate = (friction_measurement/num_measurements) + (
                        (num_measurements-1.)/num_measurements)*friction_estimate

                    # append
                    end_effector_2D_wrench_list.append(end_effector_2D_wrench)
                    generalized_velocities_list.append(generalized_velocities.data)

            # publish and sleep
            if num_measurements > 0:
                friction_estimate_message.data = friction_estimate
                robot_friction_estimate_pub.publish(friction_estimate_message)  
            else:         
                friction_estimate_message.data = NOMINAL_FRICTION_VAL
                robot_friction_estimate_pub.publish(friction_estimate_message)  

        rate.sleep()    
