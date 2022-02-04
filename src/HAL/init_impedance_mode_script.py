#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0,parentdir) 
sys.path.insert(0,gparentdir)

import collections
import pdb
import rospy
import time
import tf

from geometry_msgs.msg import PoseStamped, TransformStamped

from franka_interface import ArmInterface 
from franka_tools import CollisionBehaviourInterface

import Helpers.franka_helper as fh
import Helpers.ros_helper as rh
import Helpers.timing_helper as th
from Modelling.system_params import SystemParams


if __name__ == '__main__':

    # initialize node
    node_name = 'initialize_cartesian_impedance_mode_node'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.hal_params["RATE"])             

    # controller params
    IMPEDANCE_STIFFNESS_LIST = sys_params.controller_params[
        "IMPEDANCE_STIFFNESS_LIST"]
    
    # initialize arm
    arm = ArmInterface()
    rospy.sleep(0.5)

    # setting collision parameters
    print("Setting collision behaviour")
    collision = CollisionBehaviourInterface()
    rospy.sleep(0.5)
    torque_upper = sys_params.controller_params["TORQUE_UPPER"] 
    force_upper = sys_params.controller_params["FORCE_UPPER"]
    collision.set_ft_contact_collision_behaviour(torque_upper=torque_upper, 
        force_upper=force_upper)
    rospy.sleep(1.0)

    # cartesian impedance mode
    arm.initialize_cartesian_impedance_mode()
    arm.set_cart_impedance_pose(arm.endpoint_pose(),
                stiffness=IMPEDANCE_STIFFNESS_LIST)
    rospy.sleep(1.0)

    # while not rospy.is_shutdown():
    #     rospy.spin()