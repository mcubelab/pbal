#!/usr/bin/env python

# this is to find out the transform between the webcam frame and robot frame
import numpy as np
import tf.transformations as tfm
import rospy
import pdb
import matplotlib.pyplot as plt
import ros_helper, franka_helper

from franka_interface import ArmInterface 
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray, Float32, Bool
from models.system_params import SystemParams



def external_wrench_callback(data):
    global external_wrench
    external_wrench = data

def hand_orientation_callback(data):
    global robot_angle
    robot_angle = data.data[-1]

def torque_cone_boundary_test_callback(data):
    global torque_boundary_boolean
    torque_boundary_boolean = data.data

def update_gravity_params(theta_list, moment_list):

    theta_array = np.array(theta_list) # TODO: this should always be an array
    moment_array = np.array(moment_list)

    # build Ax = b 
    b = moment_array
    A = np.vstack([theta_array, np.ones_like(theta_array)]).T

    # solve Ax = b to find COR (x = [x0, y0, C])
    Coeff_Vec = np.linalg.lstsq(A,b)[0]

    # extract the center location for the three points chosen
    mgl = Coeff_Vec[0]
    theta0 = -Coeff_Vec[1]/Coeff_Vec[0]

    return mgl,theta0

if __name__ == '__main__':

    rospy.init_node("gravitational_params_estimator")
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.estimator_params["RATE"])

    # arm
    arm = ArmInterface()
    rospy.sleep(0.5)

    # globals
    external_wrench, robot_angle, torque_boundary_boolean = None, None, None

    # set up subscribers
    external_force_sub = rospy.Subscriber("/external_wrench_in_pivot", 
        WrenchStamped,  external_wrench_callback)

    generalized_positions_sub = rospy.Subscriber("/generalized_positions", 
        Float32MultiArray,  hand_orientation_callback)    

    torque_cone_boundary_test_sub = rospy.Subscriber("/torque_cone_boundary_test", 
        Bool,  torque_cone_boundary_test_callback)

    # set up publishers
    com_ray_pub = rospy.Publisher('/com_ray', Float32, queue_size=10)
    gravity_torque_pub = rospy.Publisher('/gravity_torque', Float32, queue_size=10)
    grav_params_vis_pub = rospy.Publisher('/grav_params_vis', WrenchStamped, queue_size=10)

    # initialize estimates
    mgl, theta0 = None, None

    # lists
    gravitational_torque_list = []
    robot_orientation_list = []

    # hyper parameters
    NBATCH = sys_params.estimator_params["NBATCH_GRAV_PARAMS"]                            # in yaml
    DELTA_ANGLE_THRESH = sys_params.estimator_params["DELTA_ANGLE_THRESH_GRAV"]           # in yaml
    ANGLE_DIFF_THRESH = sys_params.estimator_params["ANGLE_DIFF_THRESH_GRAV"]             # in yaml

    # running estimates
    max_angle, min_angle = 0., 0.
    publish_flag = False

    # empty messages
    mgl_msg, theta0_msg = Float32(), Float32()

    print('starting to estimate theta0 and mgl')
    while not rospy.is_shutdown():

        if (external_wrench is not None) and (robot_angle is not
            None) and (torque_boundary_boolean is not None):

            # get hand orientation
            external_wrench_list = ros_helper.wrench_stamped2list(external_wrench)

            # update min and max angle
            if robot_angle > max_angle:
                max_angle = robot_angle

            if robot_angle < min_angle:
                min_angle = robot_angle

            # append if empty
            if not robot_orientation_list:
                gravitational_torque_list.append(external_wrench_list[-2])
                robot_orientation_list.append(robot_angle)

            # if measured pose is new and the measurement is not at wrench cone boundary
            if (np.abs(robot_angle - robot_orientation_list[-1]) > 
                ANGLE_DIFF_THRESH) and torque_boundary_boolean:
                
                # append to list
                gravitational_torque_list.append(external_wrench_list[-2])
                robot_orientation_list.append(robot_angle)

                # make list a FIFO buffer of length NBATCH
                if len(robot_orientation_list) > NBATCH:
                    print("NBATCH long")
                    robot_orientation_list.pop(0)
                    gravitational_torque_list.pop(0)

                # and update
                if (max_angle - min_angle) > DELTA_ANGLE_THRESH:
                    mgl, theta0 = update_gravity_params(robot_orientation_list, 
                        gravitational_torque_list)
                    publish_flag = True


            # only publish if estimate has settled
            if publish_flag: 
                mgl_msg.data = mgl
                theta0_msg.data = theta0

                # grav_params_vis_msg.header.stamp = rospy.Time.now()
                grav_params_vis_msg = ros_helper.list2wrench_stamped([
                    mgl*np.sin(robot_angle-theta0), 0., 
                    mgl*np.cos(robot_angle-theta0), 0., 0., 0.])
                grav_params_vis_msg.header.frame_id = 'pivot'

                gravity_torque_pub.publish(mgl_msg)
                com_ray_pub.publish(theta0_msg)
                grav_params_vis_pub.publish(grav_params_vis_msg)

        rate.sleep()