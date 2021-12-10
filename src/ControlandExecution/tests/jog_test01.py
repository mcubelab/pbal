#!/usr/bin/env python

# this is to find out the transform between the webcam frame and robot frame
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0,parentdir) 
sys.path.insert(0,gparentdir)

import numpy as np
import tf.transformations as tfm
import tf2_ros
import rospy
import copy
import pdb
import json
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib.lines as lines
import time

from scipy.spatial import ConvexHull, convex_hull_plot_2d
import tf

from Modelling.system_params import SystemParams

import franka_helper 
import Modelling.ros_helper as ros_helper
from franka_interface import ArmInterface 
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from std_msgs.msg import Float32MultiArray, Float32, Bool, String
from franka_tools import CollisionBehaviourInterface

def get_orientation_in_base(contact_pose_homog):
    # current orientation
    hand_normal_x = contact_pose_homog[0,0]
    hand_normal_z = contact_pose_homog[2,0]

    np.array([[],[]])
    return -np.arctan2(hand_normal_x, -hand_normal_z)

def get_robot_world_xyz_theta(arm):
    
    # initial impedance target
    pose = arm.endpoint_pose()
    
    sixD_list = franka_helper.franka_pose2list(
        pose)

    theta = ros_helper.quatlist_to_theta(
        sixD_list[3:])

    xyz_theta = np.array([
        sixD_list[0],
        sixD_list[1],
        sixD_list[2],
        theta])

    return xyz_theta    

def end_effector_wrench_callback(data):
    global end_effector_wrench
    end_effector_wrench = data 

if __name__ == '__main__':


    # load params
    sys_params = SystemParams()
    controller_params = sys_params.controller_params
    initial_object_params = sys_params.object_params

    # setting collision parameters
    print("Setting collision behaviour")
    collision = CollisionBehaviourInterface()
    rospy.sleep(0.5)
    torque_upper = controller_params["TORQUE_UPPER"] 
    force_upper = controller_params["FORCE_UPPER"]
    collision.set_ft_contact_collision_behaviour(torque_upper=torque_upper, 
        force_upper=force_upper)

    IMPEDANCE_STIFFNESS_LIST = [1000, 1000, 1000, 100, 30, 100]
    
    rospy.init_node("jog_test")
    rospy.sleep(.5)

    RATE = 100.0
    rate = rospy.Rate(RATE) # in yaml


    arm = ArmInterface()




    # face_center franka pose
    endpoint_pose_franka = arm.endpoint_pose()

    # face_center list
    endpoint_pose_list = franka_helper.franka_pose2list(endpoint_pose_franka)

    contact_pose_stamped = ros_helper.list2pose_stamped(endpoint_pose_list)
    contact_pose_homog = ros_helper.matrix_from_pose(contact_pose_stamped)

    hand_theta = -get_orientation_in_base(contact_pose_homog)
    hand_position = np.array([endpoint_pose_list[0],endpoint_pose_list[2]])

    # initial impedance target
    impedance_target_pose = arm.endpoint_pose()
    impedance_target = get_robot_world_xyz_theta(arm)

    #print impedance_target

    #print impedance_target_pose['position'][0]


    impedance_target_pose['position'][2]+=.03
    arm.set_cart_impedance_pose(impedance_target_pose,
                stiffness=IMPEDANCE_STIFFNESS_LIST)

    arm.exit_control_mode()

    rospy.sleep(1)

    print hand_theta, hand_position

    

