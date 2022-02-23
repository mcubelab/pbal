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
from franka_core_msgs.msg import CartImpedanceStiffness
from Modelling.system_params import SystemParams



class impedance_mode_helper():
    def __init__(self, create_node = False):
        if create_node:
            node_name = 'cartesian_impedance_mode_helper_node'
            rospy.init_node(node_name)
            rate = rospy.Rate(100) 
            rospy.sleep(.5)

        self.Cartesian_stiffness_publisher = rospy.Publisher("/impedance_stiffness", CartImpedanceStiffness, queue_size=10)
        self.Cartesian_impedance_pose_publisher = rospy.Publisher("/equilibrium_pose", PoseStamped, queue_size=10)
        self.pose_message = PoseStamped()
        self.stiffness_message = CartImpedanceStiffness()

        rospy.on_shutdown(self._clean_shutdown)
        rospy.sleep(.5)

    def _clean_shutdown(self):
        self.Cartesian_stiffness_publisher.unregister()
        self.Cartesian_impedance_pose_publisher.unregister()

    def set_cart_impedance_pose(self, pose):
        if isinstance(pose,list):
            self.pose_message.pose.position.x = pose[0]
            self.pose_message.pose.position.y = pose[1]
            self.pose_message.pose.position.z = pose[2]
            self.pose_message.pose.orientation.x = pose[3]
            self.pose_message.pose.orientation.y = pose[4]
            self.pose_message.pose.orientation.z = pose[5]
            self.pose_message.pose.orientation.w = pose[6]
        if isinstance(pose,PoseStamped):
            self.pose_message.pose.position.x = pose.pose.position.x
            self.pose_message.pose.position.y = pose.pose.position.y 
            self.pose_message.pose.position.z = pose.pose.position.z
            self.pose_message.pose.orientation.x = pose.pose.orientation.x
            self.pose_message.pose.orientation.y = pose.pose.orientation.y
            self.pose_message.pose.orientation.z = pose.pose.orientation.z
            self.pose_message.pose.orientation.w = pose.pose.orientation.w

        self.Cartesian_impedance_pose_publisher.publish(self.pose_message)

    def set_cart_impedance_stiffness(self,stiffness=None, damping=None):
        if stiffness is not None:
            self.stiffness_message.x = stiffness[0]
            self.stiffness_message.y = stiffness[1]
            self.stiffness_message.z = stiffness[2]
            self.stiffness_message.xrot = stiffness[3]
            self.stiffness_message.yrot = stiffness[4]
            self.stiffness_message.zrot = stiffness[5]

        if damping is not None:
            self.stiffness_message.bx = damping[0]
            self.stiffness_message.by = damping[1]
            self.stiffness_message.bz = damping[2]
            self.stiffness_message.bxrot = damping[3]
            self.stiffness_message.byrot = damping[4]
            self.stiffness_message.bzrot = damping[5]
        else:
            self.stiffness_message.bx = -1.0
            self.stiffness_message.by = -1.0
            self.stiffness_message.bz = -1.0
            self.stiffness_message.bxrot = -1.0
            self.stiffness_message.byrot = -1.0
            self.stiffness_message.bzrot = -1.0

        self.Cartesian_stiffness_publisher.publish(self.stiffness_message)

    def initialize_impedance_mode(self,torque_upper,force_upper):
        # initialize arm
        arm = ArmInterface()
        rospy.sleep(0.5)

        # setting collision parameters
        print("Setting collision behaviour")
        collision = CollisionBehaviourInterface()
        rospy.sleep(0.5)
        collision.set_ft_contact_collision_behaviour(torque_upper=torque_upper, 
            force_upper=force_upper)
        rospy.sleep(1.0)

        # cartesian impedance mode
        arm.initialize_cartesian_impedance_mode()
        rospy.sleep(1.0)



    
    

    