#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import rospy
import tf

import Helpers.ros_helper as rh
from Helpers.time_logger import time_logger
from Modelling.system_params import SystemParams
from Helpers.ros_manager import ros_manager

if __name__ == '__main__':

    # initialize node
    node_name = 'ee_pose_transformations'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.hal_params["RATE"])

    listener = tf.TransformListener()
    
    # define publishers
    rm = ros_manager()

    rm.spawn_publisher_list(['/ee_pose_in_base_from_franka_publisher',
                             '/ee_pose_in_world_manipulation_from_franka_publisher'])

    # object for computing loop frequency
    tl = time_logger(node_name)

    #5. Run node at rate
    while not rospy.is_shutdown():
        tl.reset()
 
        (ee_pose_world_manipulation_trans, ee_pose_world_manipulation_rot) = rh.lookupTransform('/panda_EE', '/world_manipulation_frame', listener)
        (ee_pose_base_trans, ee_pose_base_rot) = rh.lookupTransform('/panda_EE', 'base', listener)

        if ee_pose_world_manipulation_trans is not None and ee_pose_world_manipulation_rot is not None:
            ee_pose_in_world_manipulation_list = ee_pose_world_manipulation_trans+ee_pose_world_manipulation_rot

            # publish
            rm.pub_ee_pose_in_world_manipulation_from_franka(ee_pose_in_world_manipulation_list)

        if ee_pose_base_trans is not None and ee_pose_base_rot is not None:
            ee_pose_in_base_list = ee_pose_base_trans+ee_pose_base_rot

            # publish
            rm.pub_ee_pose_in_base_from_franka(ee_pose_in_base_list)

        # log timing info       
        tl.log_time()

        rate.sleep()