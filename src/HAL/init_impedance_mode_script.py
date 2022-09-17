#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,s.path.dirname(os.path.dirname(currentdir)))

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
import Helpers.impedance_mode_helper as IMH
from Modelling.system_params import SystemParams

if __name__ == '__main__':

    sys_params = SystemParams()
    # controller params
    IMPEDANCE_STIFFNESS_LIST = sys_params.controller_params[
        "IMPEDANCE_STIFFNESS_LIST"]

    # IMPEDANCE_STIFFNESS_LIST = [100.0, 100.0, 100.0, 10.0, 10.0, 10.0]
    torque_upper = sys_params.controller_params["TORQUE_UPPER"] 
    force_upper = sys_params.controller_params["FORCE_UPPER"]

    # IMH.test_set_impedance_with_topic(IMPEDANCE_STIFFNESS_LIST)
    my_impedance_mode_helper = IMH.impedance_mode_helper(True)
    my_impedance_mode_helper.initialize_impedance_mode(torque_upper,force_upper)
    # my_impedance_mode_helper.set_cart_impedance_stiffness(IMPEDANCE_STIFFNESS_LIST)
