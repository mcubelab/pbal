#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import time

import Helpers.kinematics_helper as kh
from Helpers.ros_manager import ros_manager

def generate_winding_indices(height,width):
    my_E_matrix = np.zeros([height,width])
    index_list = []
    index_A=0
    index_B=1
    
    count = 1

    index_list.append([0,0])
    while count<=(width-1)*height:
        my_E_matrix[index_A,index_B]=count
        
        index_list.append([index_A,index_B])
        if index_A%2==0:
            if index_B==width-1:
                index_A+=1
            else:
                index_B+=1
        else:
            if index_B==1:
                index_A+=1
            else:
                index_B-=1
        count+=1

    index_A-=1
    while index_A>0:
        my_E_matrix[index_A,0]=count
        index_list.append([index_A,0])
        count+=1
        index_A-=1

    return index_list

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    
    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = np.dot(np.transpose(AA), BB)

    U, S, Vt = np.linalg.svd(H)

    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       print 'Reflection detected'
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)

    t = -np.dot(R,centroid_A.T) + centroid_B.T
    
    ret_R = R
    ret_t = t
    A2 = np.dot(ret_R,A.T) + np.tile(ret_t, (N, 1)).T
    A2 = A2.T

    # Find the error
    err = A2 - B

    err = np.multiply(err, err)
    err = np.sum(err)
    rmse = np.sqrt(err/N);

    return R, t, rmse

if __name__ == '__main__':
    cam_to_calibrate = 'near'
    # cam_to_calibrate = 'far'
    # cam_to_calibrate = 'wherever you are'

    apriltag_id = 10
    
    rm = ros_manager()
    rm.init_node('realsense_live_calibration_test')
    rm.setRate(10)

    rm.spawn_transform_listener()
    rm.impedance_mode_helper()

    rm.subscribe_to_list(['/tag_detections',
                          '/ee_pose_in_base_from_franka_publisher'])


    # initialize impedance mode
    IMPEDANCE_STIFFNESS_LIST = [3000, 3000, 3000, 100, 60, 100] 
    #the damping values for the cartesian impedance, also a diagonal matrix
    IMPEDANCE_DAMPING_LIST = [0.5 * np.sqrt(k) for k in IMPEDANCE_STIFFNESS_LIST] 

    rm.set_cart_impedance_stiffness(IMPEDANCE_STIFFNESS_LIST,IMPEDANCE_DAMPING_LIST)
 
    rm.wait_for_necessary_data()


    l_tag = .042
    x_tag_boundary = np.array([-l_tag/2,l_tag/2,l_tag/2,-l_tag/2])
    y_tag_boundary = np.array([l_tag/2,l_tag/2,-l_tag/2,-l_tag/2])
    z_tag_boundary = np.array([0.0]*4)
    one_tag_boundary = np.array([1.0]*4)

    tag_boundary_pts = np.vstack([x_tag_boundary,y_tag_boundary,z_tag_boundary,one_tag_boundary])

    if cam_to_calibrate == 'far':
        desired_pose_homog = np.array([ [ 0.0,-1.0, 0.0, 0.5],
                                        [ 0.0, 0.0, 1.0, 0.25],
                                        [-1.0, 0.0, 0.0, 0.15],
                                        [ 0.0, 0.0, 0.0, 1.0]])
        min_X = .32
        max_X = .58

        min_Y = .05
        max_Y = .3

        min_Z = .07
        max_Z = .17

    if cam_to_calibrate == 'near':
        desired_pose_homog = np.array([ [ 0.0, 1.0, 0.0, 0.5],
                                        [ 0.0, 0.0,-1.0,-0.25],
                                        [-1.0, 0.0, 0.0, 0.15],
                                        [ 0.0, 0.0, 0.0, 1.0]])
        min_X = .32
        max_X = .58

        min_Y = -.05
        max_Y = -.3

        min_Z = .07
        max_Z = .17

    candidate_points_start =  np.array([
        [ 1.0,-1.0, 0.0, 0.0, 0.0, 0.0],
        [ 0.0, 0.0, 1.0,-1.0, 0.0, 0.0],
        [ 0.0, 0.0, 0.0, 0.0, 1.0,-1.0],
        [ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])

    candidate_points_start[0:3,:]*=.21

    april_tag_pts_from_robot = np.zeros([3,0])
    april_tag_pts_from_camera = np.zeros([3,0])

    winding_height = 4
    winding_width = 4
    winding_depth = 4
    num_waypoints = winding_height*winding_width

    X_range = np.linspace(min_X,max_X,winding_width)
    Y_range = np.linspace(max_Y,min_Y,winding_depth)
    Z_range = np.linspace(min_Z,max_Z,winding_height)


    my_winding_indices = generate_winding_indices(winding_height,winding_width)

    count = 0

    rm.ee_pose_in_base_unpack()

    x_current = rm.ee_pose_in_base_list[0]
    y_current = rm.ee_pose_in_base_list[1]
    z_current = rm.ee_pose_in_base_list[2]

    x_start = X_range[0]
    y_start = Y_range[0]
    z_start = Z_range[0]

    t0 = time.time()
    step_time = 3.0

    while time.time()-t0<step_time:
        alpha = min(1.0,(time.time()-t0)/step_time)

        x_target = alpha*x_start + (1-alpha)*x_current
        y_target = alpha*y_start + (1-alpha)*y_current
        z_target = alpha*z_start + (1-alpha)*z_current

        desired_pose_homog[0,3] = x_target
        desired_pose_homog[1,3] = y_target
        desired_pose_homog[2,3] = z_target

        rm.set_cart_impedance_pose(kh.pose_list_from_matrix(desired_pose_homog))

    desired_pose_homog[0,3] = x_start
    desired_pose_homog[1,3] = y_start
    desired_pose_homog[2,3] = z_start

    t0 = time.time()
    step_time = 1.5

    snapshot_taken = False
    snapshot_threshold =1.9

    while not rm.is_shutdown():
        rm.unpack_all()

        t = time.time()-t0
        next_index = np.ceil(t/step_time)
        current_index = np.floor(t/step_time)

        alpha_raw = (t-current_index*step_time)/(.5*step_time)
        alpha = min(1,alpha_raw)
        
        if int(next_index)//num_waypoints>=len(Z_range):
            break

        z_target_next = Z_range[my_winding_indices[int(next_index)%num_waypoints][0]]
        y_target_next = Y_range[int(next_index)//num_waypoints]
        x_target_next = X_range[my_winding_indices[int(next_index)%num_waypoints][1]]

        z_target_current = Z_range[my_winding_indices[int(current_index)%num_waypoints][0]]
        y_target_current = Y_range[int(current_index)//num_waypoints]
        x_target_current = X_range[my_winding_indices[int(current_index)%num_waypoints][1]]

        z_target = alpha*z_target_next+(1-alpha)*z_target_current
        y_target = alpha*y_target_next+(1-alpha)*y_target_current
        x_target = alpha*x_target_next+(1-alpha)*x_target_current
        
        desired_pose_homog[0,3] = x_target
        desired_pose_homog[1,3] = y_target
        desired_pose_homog[2,3] = z_target

        rm.set_cart_impedance_pose(kh.pose_list_from_matrix(desired_pose_homog))

        if alpha_raw<=snapshot_threshold:
            snapshot_taken = False


        if apriltag_id in rm.apriltag_pose_homog_dict:
            (cam_in_base_trans, cam_in_base_rot) = rm.lookupTransform('/panda_april_tag', 'base')

            robot_apriltag_pose_matrix = kh.matrix_from_trans_and_quat(cam_in_base_trans,cam_in_base_rot)
            
            robot_apriltag_rot = robot_apriltag_pose_matrix[0:3,0:3]
            robot_apriltag_trans = robot_apriltag_pose_matrix[0:3,3]

            obj_pose_homog = rm.apriltag_pose_homog_dict[apriltag_id]

            camera_apriltag_rot = obj_pose_homog[0:3,0:3]
            camera_apriltag_trans = obj_pose_homog[0:3,3]

            if alpha_raw>snapshot_threshold:
                if not snapshot_taken:
                    camera_points =  np.dot(obj_pose_homog,candidate_points_start)
                    robot_points =  np.dot(robot_apriltag_pose_matrix,candidate_points_start)

                    april_tag_pts_from_robot = np.hstack([april_tag_pts_from_robot,robot_points[0:3,:]])
                    april_tag_pts_from_camera = np.hstack([april_tag_pts_from_camera,camera_points[0:3,:]])

                    snapshot_taken = True

    # then you'll get webcam frame wrt robot frame
    (R, t, rmse) = rigid_transform_3D(np.transpose(april_tag_pts_from_camera), np.transpose(april_tag_pts_from_robot))  
    camera_pose = kh.pose_list_from_matrix(np.vstack([np.hstack([np.array(R),np.transpose(np.array([t]))]),np.array([[0.0,0.0,0.0,1.0]])]) )

    print(R)
    print(t)
    print(rmse)
    print(camera_pose)
    