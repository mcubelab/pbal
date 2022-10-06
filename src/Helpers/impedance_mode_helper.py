#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from franka_interface import ArmInterface 
from franka_tools import CollisionBehaviourInterface
import Helpers.ros_helper as rh
from franka_core_msgs.msg import CartImpedanceStiffness

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

    def set_matrices_pbal_mode(self,TIPI,TOOPI,RIPI,ROOPI,rot_mat):

        IMPEDANCE_STIFFNESS_TRANS = np.array([[ TIPI,    0.0,    0.0],
                                              [  0.0,   TIPI,    0.0],
                                              [  0.0,    0.0,  TOOPI]])

        IMPEDANCE_STIFFNESS_ROT   = np.array([[ ROOPI,   0.0,   0.0],
                                              [   0.0, ROOPI,   0.0],
                                              [   0.0,   0.0,  RIPI]])


        IMPEDANCE_DAMPING_TRANS   = np.array([[ .5*np.sqrt(TIPI),              0.0,               0.0],
                                              [              0.0, .5*np.sqrt(TIPI),               0.0],
                                              [              0.0,              0.0, .5*np.sqrt(TOOPI)]])

        IMPEDANCE_DAMPING_ROT     = np.array([[ .5*np.sqrt(RIPI),              0.0,               0.0],
                                              [              0.0, .5*np.sqrt(RIPI),               0.0],
                                              [              0.0,              0.0, .5*np.sqrt(ROOPI)]])

        IMPEDANCE_STIFFNESS_TRANS = np.dot(np.dot(rot_mat,IMPEDANCE_STIFFNESS_TRANS),rot_mat.T)
        IMPEDANCE_STIFFNESS_ROT   = np.dot(np.dot(rot_mat,IMPEDANCE_STIFFNESS_ROT),rot_mat.T)
        IMPEDANCE_DAMPING_TRANS   = np.dot(np.dot(rot_mat,IMPEDANCE_DAMPING_TRANS),rot_mat.T)
        IMPEDANCE_DAMPING_ROT     = np.dot(np.dot(rot_mat,IMPEDANCE_DAMPING_ROT),rot_mat.T)

        self.set_cart_imepedance_stiffness_matrix(stiffness_trans=IMPEDANCE_STIFFNESS_TRANS, 
                                                   stiffness_rot=IMPEDANCE_STIFFNESS_ROT, 
                                                   damping_trans=IMPEDANCE_DAMPING_TRANS,   
                                                   damping_rot=IMPEDANCE_DAMPING_ROT)

    def set_cart_impedance_pose(self, pose):

        self.stiffness_message.use_flag = 0

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

        self.stiffness_message.use_flag = 0

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


    def set_cart_imepedance_stiffness_matrix(self,stiffness_trans=None, stiffness_rot=None, 
                                                  damping_trans=None,   damping_rot=None):
        
        if stiffness_trans is not None and stiffness_rot is not None and damping_trans is not None and damping_rot is not None:

            self.stiffness_message.use_flag = 1

            self.stiffness_message.xx = stiffness_trans[0][0]
            self.stiffness_message.xy = stiffness_trans[0][1]
            self.stiffness_message.xz = stiffness_trans[0][2]
            self.stiffness_message.yx = stiffness_trans[1][0]
            self.stiffness_message.yy = stiffness_trans[1][1]
            self.stiffness_message.yz = stiffness_trans[1][2]
            self.stiffness_message.zx = stiffness_trans[2][0]
            self.stiffness_message.zy = stiffness_trans[2][1]
            self.stiffness_message.zz = stiffness_trans[2][2]

            self.stiffness_message.xxrot = stiffness_rot[0][0] 
            self.stiffness_message.xyrot = stiffness_rot[0][1]
            self.stiffness_message.xzrot = stiffness_rot[0][2]
            self.stiffness_message.yxrot = stiffness_rot[1][0]
            self.stiffness_message.yyrot = stiffness_rot[1][1]
            self.stiffness_message.yzrot = stiffness_rot[1][2]
            self.stiffness_message.zxrot = stiffness_rot[2][0]
            self.stiffness_message.zyrot = stiffness_rot[2][1]
            self.stiffness_message.zzrot = stiffness_rot[2][2]
            
            self.stiffness_message.bxx = damping_trans[0][0]
            self.stiffness_message.bxy = damping_trans[0][1]
            self.stiffness_message.bxz = damping_trans[0][2]
            self.stiffness_message.byx = damping_trans[1][0]
            self.stiffness_message.byy = damping_trans[1][1]
            self.stiffness_message.byz = damping_trans[1][2]
            self.stiffness_message.bzx = damping_trans[2][0]
            self.stiffness_message.bzy = damping_trans[2][1]
            self.stiffness_message.bzz = damping_trans[2][2]
            
            self.stiffness_message.bxxrot = damping_rot[0][0]
            self.stiffness_message.bxyrot = damping_rot[0][1]
            self.stiffness_message.bxzrot = damping_rot[0][2]
            self.stiffness_message.byxrot = damping_rot[1][0]
            self.stiffness_message.byyrot = damping_rot[1][1]
            self.stiffness_message.byzrot = damping_rot[1][2]
            self.stiffness_message.bzxrot = damping_rot[2][0]
            self.stiffness_message.bzyrot = damping_rot[2][1]
            self.stiffness_message.bzzrot = damping_rot[2][2]

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



    
    

    