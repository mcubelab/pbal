#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

from Modelling.system_params import SystemParams
from Helpers.ros_manager import ros_manager

if __name__ == '__main__':

    # initialize node
    node_name = 'ee_pose_transformations'
    sys_params = SystemParams()

    
    
    # define publishers
    rm = ros_manager()
    rm.init_node(node_name)
    rm.setRate(sys_params.estimator_params["RATE"])
    rm.spawn_publisher_list(['/ee_pose_in_base_from_franka_publisher',
                             '/ee_pose_in_world_manipulation_from_franka_publisher'])

    rm.spawn_transform_listener()

    # object for computing loop frequency
    rm.init_time_logger()

    #5. Run node at rate
    while not rm.is_shutdown():
        rm.tl_reset()
 
        (ee_pose_world_manipulation_trans, ee_pose_world_manipulation_rot) = rm.lookupTransform('/panda_EE', '/world_manipulation_frame')
        (ee_pose_base_trans, ee_pose_base_rot) = rm.lookupTransform('/panda_EE', 'base')

        if ee_pose_world_manipulation_trans is not None and ee_pose_world_manipulation_rot is not None:
            ee_pose_in_world_manipulation_list = ee_pose_world_manipulation_trans+ee_pose_world_manipulation_rot

            # publish
            rm.pub_ee_pose_in_world_manipulation_from_franka(ee_pose_in_world_manipulation_list)

        if ee_pose_base_trans is not None and ee_pose_base_rot is not None:
            ee_pose_in_base_list = ee_pose_base_trans+ee_pose_base_rot

            # publish
            rm.pub_ee_pose_in_base_from_franka(ee_pose_in_base_list)

        # log timing info       
        rm.log_time()
        rm.sleep()