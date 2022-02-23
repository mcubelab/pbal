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
    node_name = 'ee_pose_in_world_from_franka'
    rospy.init_node(node_name)
    sys_params = SystemParams()
    rate = rospy.Rate(sys_params.hal_params["RATE"])

    listener = tf.TransformListener()
    
    # define publishers
    ee_pose_in_world_from_franka_pub = rospy.Publisher(
        '/ee_pose_in_world_from_franka_publisher', 
        PoseStamped, queue_size = 10)

    # queue for computing frequnecy
    time_deque = collections.deque(maxlen=sys_params.debug_params['QUEUE_LEN'])


    #5. Run node at rate
    while not rospy.is_shutdown():

        t0 = time.time()
 
        (ee_pose_world_trans, ee_pose_world_rot) = rh.lookupTransform('/panda_EE', 'base', listener)
        ee_pose_in_world_list = ee_pose_world_trans+ee_pose_world_rot
        ee_pose_in_world_pose_stamped = rh.list2pose_stamped(ee_pose_in_world_list)

        # publish and sleep
        ee_pose_in_world_from_franka_pub.publish(ee_pose_in_world_pose_stamped)

        # update time deque
        time_deque.append(1000 * (time.time() - t0))   

        # log timing info
        if len(time_deque) == sys_params.debug_params['QUEUE_LEN']:
            rospy.loginfo_throttle(sys_params.debug_params["LOG_TIME"], 
                (node_name + " runtime: {mean:.3f} +/- {std:.3f} [ms]")
                .format(mean=sum(time_deque)/len(time_deque), 
                std=th.compute_std_dev(my_deque=time_deque, 
                    mean_val=sum(time_deque)/len(time_deque))))


        rate.sleep()