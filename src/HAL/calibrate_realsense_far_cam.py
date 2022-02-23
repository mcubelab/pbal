#!/usr/bin/env python
import os, sys, inspect

currentdir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
gparentdir = os.path.dirname(parentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, gparentdir)

import copy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from cvxopt import matrix, solvers
import json
import numpy as np
import pdb
import rospy
import time
import tf
import tf.transformations as tfm

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped, PoseStamped, WrenchStamped
from pbal.msg import FrictionParamsStamped, QPDebugStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray

from Modelling.system_params import SystemParams
import Helpers.ros_helper as rh
import Helpers.pbal_msg_helper as pmh
import Helpers.impedance_mode_helper as IMH

def initialize_frame():
    frame_message = TransformStamped()
    frame_message.header.frame_id = "base"
    frame_message.header.stamp = rospy.Time.now()
    frame_message.child_frame_id = "hand_estimate"
    frame_message.transform.translation.x = 0.0
    frame_message.transform.translation.y = 0.0
    frame_message.transform.translation.z = 0.0

    frame_message.transform.rotation.x = 0.0
    frame_message.transform.rotation.y = 0.0
    frame_message.transform.rotation.z = 0.0
    frame_message.transform.rotation.w = 1.0
    return frame_message

def update_frame(frame_pose_stamped, frame_message):
    frame_message.header.stamp = rospy.Time.now()
    frame_message.transform.translation.x = frame_pose_stamped.pose.position.x
    frame_message.transform.translation.y = frame_pose_stamped.pose.position.y
    frame_message.transform.translation.z = frame_pose_stamped.pose.position.z
    frame_message.transform.rotation.x = frame_pose_stamped.pose.orientation.x
    frame_message.transform.rotation.y = frame_pose_stamped.pose.orientation.y
    frame_message.transform.rotation.z = frame_pose_stamped.pose.orientation.z
    frame_message.transform.rotation.w = frame_pose_stamped.pose.orientation.w

def robot2_pose_list(xyz_list, theta):
    return xyz_list + rh.theta_to_quatlist(theta)

def apriltag_message_callback(apriltag_array):
    global apriltag_id

    global object_detected
    global obj_pose_homog

    obj_apriltag_list = [
        detection for detection in apriltag_array.detections
        if detection.id == (apriltag_id, )
    ]

    if obj_apriltag_list:
        object_detected = True

        obj_pose_homog = rh.matrix_from_pose(obj_apriltag_list[0].pose.pose)

    else:
        object_detected = False


# def camera_info_callback(data):
#     global camera_info

#     camera_info = data

# def get_pix(xyz, transformation_matrix):
#     # xyz should be n x 3

#     # Project trajectories into pixel space
#     vector = np.hstack([xyz, np.ones([xyz.shape[0], 1])])
#     pixels = np.dot(transformation_matrix, vector.T)
#     pix_x = np.divide(pixels[0], pixels[2])
#     pix_y = np.divide(pixels[1], pixels[2])

#     return pix_x, pix_y

def ee_pose_callback(data):
    global panda_hand_in_base_pose
    panda_hand_in_base_pose = data

def get_pix_easier(xyz, transformation_matrix):
    pixels = np.dot(transformation_matrix, xyz)
    pix_x = np.divide(pixels[0], pixels[2])
    pix_y = np.divide(pixels[1], pixels[2])

    return np.round(pix_x).astype(int), np.round(pix_y).astype(int)

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
       print "Reflection detected"
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

    panda_hand_in_base_pose = None

    apriltag_id = 10
    object_detected = False
    obj_pose_homog = None

    camera_info = None

    image_list = []

    l_tag = .042
    x_tag_boundary = np.array([-l_tag/2,l_tag/2,l_tag/2,-l_tag/2])
    y_tag_boundary = np.array([l_tag/2,l_tag/2,-l_tag/2,-l_tag/2])
    z_tag_boundary = np.array([0.0]*4)
    one_tag_boundary = np.array([1.0]*4)

    tag_boundary_pts = np.vstack([x_tag_boundary,y_tag_boundary,z_tag_boundary,one_tag_boundary])
  

    
    


    rospy.init_node("realsense_live_calibration_test")
    rate = rospy.Rate(10)
    rospy.sleep(1.0)


    # initialize impedance mode
    IMPEDANCE_STIFFNESS_LIST = [3000, 3000, 3000, 100, 60, 100] #[100, 100, 100, 100, 100, 100] #[50, 50, 50, 10, 10, 10] #[2000, 2000, 2000, 200, 60, 200]
  
    
    my_impedance_mode_helper = IMH.impedance_mode_helper()
    my_impedance_mode_helper.set_cart_impedance_stiffness(IMPEDANCE_STIFFNESS_LIST)


    listener = tf.TransformListener()
 
    bridge = CvBridge()

    # image_message_sub = rospy.Subscriber('/far_cam/color/image_raw', Image,
    #                                      image_message_callback)

    # camera_info_sub = rospy.Subscriber('/far_cam/color/camera_info', CameraInfo,
    #                                    camera_info_callback)

    apriltag_message_sub = rospy.Subscriber('/tag_detections',
                                            AprilTagDetectionArray,
                                            apriltag_message_callback)
        # subscribers
    panda_hand_in_base_pose_sub  = rospy.Subscriber(
                                        '/ee_pose_in_world_from_franka_publisher',
                                        PoseStamped,
                                        ee_pose_callback,
                                        queue_size=1)

    # intialize impedance target frame
    frame_message = initialize_frame()
    target_frame_pub = rospy.Publisher('/target_frame', 
                                        TransformStamped, 
                                        queue_size=10) 

    rospy.sleep(1.0)
    # print("Waiting for image message")
    # while len(image_list) == 0:
    #     rospy.sleep(0.1)

    # print("Waiting for camera info")
    # while camera_info is None:
    #     rospy.sleep(.1)

    # # (3, 4) camera matrix
    # camera_matrix = np.reshape(camera_info.P, (3, 4))
    # camera_info_sub.unregister()

    M1 = np.zeros([3,3])
    M2 = np.zeros(3)
    forgetting_factor = .01

    trans_diff = np.zeros(3)
    camera_pose_mat = np.zeros([4,4])
    camera_pose_mat[3,3]=1.0

    cam_to_calibrate = 'near'
    # cam_to_calibrate = 'far'

    if cam_to_calibrate == 'far':
        desired_pose_homog = np.array([ [ 0.0,-1.0, 0.0, 0.5],
                                        [ 0.0, 0.0, 1.0, 0.25],
                                        [-1.0, 0.0, 0.0, 0.15],
                                        [ 0.0, 0.0, 0.0, 1.0]])

        min_X = .32
        max_X = .62

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
        max_X = .62

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

    # print candidate_points_start

    april_tag_pts_from_robot = np.zeros([3,0])
    april_tag_pts_from_camera = np.zeros([3,0])

    winding_height = 8
    winding_width = 8
    winding_depth = 8
    num_waypoints = winding_height*winding_width



    X_range = np.linspace(min_X,max_X,winding_width)
    Y_range = np.linspace(max_Y,min_Y,winding_depth)
    Z_range = np.linspace(min_Z,max_Z,winding_height)


    my_winding_indices = generate_winding_indices(winding_height,winding_width)

    count = 0



    starting_pose = rh.matrix_from_pose(panda_hand_in_base_pose)

    x_current = starting_pose[0,3]
    y_current = starting_pose[1,3]
    z_current = starting_pose[2,3]

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

        my_impedance_mode_helper.set_cart_impedance_pose(rh.pose_from_matrix(desired_pose_homog))




    desired_pose_homog[1,3] = y_start

    t0 = time.time()
    step_time = 1.5

    snapshot_taken = False
    snapshot_threshold =1.9

    while not rospy.is_shutdown():
       
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





        my_impedance_mode_helper.set_cart_impedance_pose(rh.pose_from_matrix(desired_pose_homog))

        if alpha_raw<=snapshot_threshold:
            snapshot_taken = False

        if object_detected:

            (cam_in_base_trans, cam_in_base_rot) = rh.lookupTransform(
                '/panda_april_tag', 'base', listener)

            robot_apriltag_pose_matrix = rh.matrix_from_trans_and_quat(cam_in_base_trans,cam_in_base_rot)
            
            robot_apriltag_rot = robot_apriltag_pose_matrix[0:3,0:3]
            robot_apriltag_trans = robot_apriltag_pose_matrix[0:3,3]

            camera_apriltag_rot = obj_pose_homog[0:3,0:3]
            camera_apriltag_trans = obj_pose_homog[0:3,3]

            if alpha_raw>snapshot_threshold:
                if not snapshot_taken:
                    camera_points =  np.dot(obj_pose_homog,candidate_points_start)
                    robot_points =  np.dot(robot_apriltag_pose_matrix,candidate_points_start)

                    april_tag_pts_from_robot = np.hstack([april_tag_pts_from_robot,robot_points[0:3,:]])
                    april_tag_pts_from_camera = np.hstack([april_tag_pts_from_camera,camera_points[0:3,:]])

                    snapshot_taken = True


    (R, t, rmse) = rigid_transform_3D(np.transpose(april_tag_pts_from_camera), np.transpose(april_tag_pts_from_robot))  # then you'll get webcam frame wrt robot frame

    print R
    print t
    print rmse

    print rh.pose_stamped2list(rh.pose_from_matrix(
        np.vstack([np.hstack([np.array(R),np.transpose(np.array([t]))]),
            np.array([[0.0,0.0,0.0,1.0]])])))

        # print type(rh.pose_from_matrix(desired_pose_homog))
        # print isinstance(rh.pose_from_matrix(desired_pose_homog),PoseStamped)
        # target_frame_pub.publish(frame_message)

            # M1 = (1-forgetting_factor)*M1 + forgetting_factor*np.dot(robot_apriltag_rot,np.transpose(robot_apriltag_rot))
            # M2 = (1-forgetting_factor)*M2 + forgetting_factor*np.dot(camera_apriltag_rot,np.transpose(robot_apriltag_rot))


            # rot_mat_out = np.linalg.solve(M1,M2)
            # trans_diff = (1-forgetting_factor)*trans_diff + forgetting_factor*(camera_apriltag_trans-np.dot(rot_mat_out,robot_apriltag_trans))

            # camera_pose_mat[0:3,0:3]=rot_mat_out
            # camera_pose_mat[0:3,3]=trans_diff
            # # print np.dot(rot_mat_out,robot_apriltag_rot)-camera_apriltag_rot
            # # print trans_diff - (np.dot(rot_mat_out,robot_apriltag_trans)-camera_apriltag_trans)

            # cv2.polylines(cv_image, [np.vstack([x_coord, y_coord]).T],
            #     True, (0, 255, 0),thickness=2)

            # x_coord, y_coord = get_pix_easier(np.dot(camera_pose_mat,np.dot(robot_apriltag_pose_matrix,tag_boundary_pts)),camera_matrix)
            # cv2.polylines(cv_image, [np.vstack([x_coord, y_coord]).T],
            #     True, (255, 0, 0),thickness=2)

        
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)
        # rate.sleep()
