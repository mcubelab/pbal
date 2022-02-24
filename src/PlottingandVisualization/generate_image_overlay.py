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
import pickle

from Modelling.system_params import SystemParams
import Estimation.friction_reasoning as friction_reasoning


def pose_list_to_matrix(pose_in):
    rotation_matrix = quat_to_mat(pose_in[3:7])
    translation_vector = np.array([pose_in[0:3]])
    return np.vstack([
        np.hstack([rotation_matrix,np.transpose(translation_vector)]),
        np.array([0.0,0.0,0.0,1.0])])

#This function was copy-pasted from the Automatic Addison website
def quat_to_mat(quat_in):
    x = quat_in[0]
    y = quat_in[1]
    z = quat_in[2]
    w = quat_in[3]

    r00 = 2*(w*w+x*x)-1
    r01 = 2*(x*y-w*z)
    r02 = 2*(x*z+w*y)

    r10 = 2*(x*y+w*z)
    r11 = 2*(w*w+y*y)-1
    r12 = 2*(y*z-w*x)
     
    r20 = 2*(x*z-w*y)
    r21 = 2*(y*z+w*x)
    r22 = 2*(w*w+z*z)-1
     
    return  np.array([[r00, r01, r02],
                      [r10, r11, r12],
                      [r20, r21, r22]])
                            

def load_shape_data(name_in):
    curr_dir = os.path.dirname(
        os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(curr_dir)
    gparentdir = os.path.dirname(parentdir)
    print 'parentdir', parentdir
    fname = os.path.join(parentdir, 'Modelling', 'shape_description',
                         name_in + ".json")
    f = open(fname)
    # f = open("/home/oneills/Documents/panda/base_ws/src/franka_ros_interface/franka_ros_controllers/scripts/models/shape_description/"+name_in+".json")
    shape_data = json.load(f)

    vertex_array = np.array([shape_data["x_vert"], shape_data["y_vert"]])
    apriltag_id = shape_data["apriltag_id"]
    apriltag_pos = shape_data["centroid_to_apriltag"]

    return vertex_array, apriltag_id, apriltag_pos

def get_pix(xyz, camera_transformation_matrix):
    # xyz should be n x 3

    # Project trajectories into pixel space
    vector = np.hstack([xyz, np.ones([xyz.shape[0], 1])])
    pixels = np.dot(camera_transformation_matrix, vector.T)
    pix_x = np.divide(pixels[0], pixels[2])
    pix_y = np.divide(pixels[1], pixels[2])

    return pix_x, pix_y

def get_pix_easier(xyz, camera_transformation_matrix):
    pixels = np.dot(camera_transformation_matrix, xyz)
    pix_x = np.divide(pixels[0], pixels[2])
    pix_y = np.divide(pixels[1], pixels[2])

    return np.round(pix_x).astype(int), np.round(pix_y).astype(int)

def enumerate_vertices_of_constraint_polygon(theta_list, b_list, closed=True):

    if len(theta_list) < 3:
        return np.array([]), np.array([])

    num_constraints = len(theta_list)
    A = np.zeros([num_constraints, 2])
    B = np.array(b_list) + (1e-7)

    vertex_x_list = []
    vertex_y_list = []

    #build constraint matrix for the polygon
    for i in range(num_constraints):
        A[i][0] = np.cos(theta_list[i])
        A[i][1] = np.sin(theta_list[i])

    theta_A = theta_list[-1]
    theta_B = theta_list[0]
    while theta_A - theta_B > np.pi:
        theta_A = theta_A - 2 * np.pi
    while theta_A - theta_B < -np.pi:
        theta_A = theta_A + 2 * np.pi

    theta_start = (theta_A + theta_B) / 2
    cost_start = np.array([-np.cos(theta_start), -np.sin(theta_start)])

    solvers.options['show_progress'] = False
    sol = solvers.lp(matrix(cost_start), matrix(A), matrix(B))

    # print np.transpose(np.array([theta_list,b_list]))
    if sol['x'] is None:
        return np.array([]), np.array([])

    #find the first vertex
    x_sol = np.squeeze(sol['x'])
    vertex_x_list.append(x_sol[0])
    vertex_y_list.append(x_sol[1])

    #active constraints for the first vertex
    dual_list = np.squeeze(sol['z'])
    dual_value_index_list = np.argsort(dual_list)
    # print np.sort(np.squeeze(sol['z']))

    if len(dual_value_index_list) < 3:
        return np.array([]), np.array([])

    #find the active constraint for the first vertex
    # index_CW  = np.max([dual_value_index_list[-1],dual_value_index_list[-2]])
    # index_CCW = np.min([dual_value_index_list[-1],dual_value_index_list[-2]])

    theta_copy = copy.deepcopy(theta_list)
    CW_index_list = []
    CW_dual_list = []
    CCW_index_list = []
    CCW_dual_list = []

    for i in range(len(theta_copy)):
        while theta_copy[i] - theta_start > np.pi:
            theta_copy[i] -= 2 * np.pi
        while theta_copy[i] - theta_start < -np.pi:
            theta_copy[i] += 2 * np.pi

        if theta_copy[i] > theta_start:
            CCW_index_list.append(i)
            CCW_dual_list.append(dual_list[i])
        if theta_copy[i] < theta_start:
            CW_index_list.append(i)
            CW_dual_list.append(dual_list[i])
    CW_dual_sorted_args = np.argsort(CW_dual_list)
    CCW_dual_sorted_args = np.argsort(CCW_dual_list)

    index_CW = CW_index_list[CW_dual_sorted_args[-1]]
    index_CCW = CCW_index_list[CCW_dual_sorted_args[-1]]

    vertex_index = [index_CW, index_CCW]

    # print [index_CW, index_CCW,len(A)]
    current_index_0 = index_CCW
    current_index_1 = current_index_0 + 1

    stopping_index = index_CW

    while current_index_1 <= stopping_index:
        test_vertex = np.linalg.solve(A[[current_index_0, current_index_1]],
                                      B[[current_index_0, current_index_1]])

        interior_check = np.dot(A, test_vertex) < B
        interior_check[[current_index_0, current_index_1]] = True

        if all(interior_check):
            vertex_x_list.append(test_vertex[0])
            vertex_y_list.append(test_vertex[1])
            current_index_0 = current_index_1
            vertex_index.append(current_index_1)

        current_index_1 = current_index_1 + 1

    # print('length of vertex list:', len(vertex_x_list))
    # print('vertex_index_sequence:', vertex_index)
    # print('optimization solution:', np.squeeze(sol['x']))

    return np.array(vertex_x_list), np.array(vertex_y_list)




def estimate_ground_COP(obj_vertices_world,measured_base_wrench_6D, height_threshold = .003):
    P0 = None

    if measured_base_wrench_6D is not None and obj_vertices_world is not None:
        height_indices = np.argsort(obj_vertices_world[2])
        if obj_vertices_world[2][height_indices[1]] - obj_vertices_world[2][height_indices[0]] < height_threshold:
            P_a = obj_vertices_world[:, height_indices[0]]
            P_b = obj_vertices_world[:, height_indices[1]]
            P_e = contact_pose_homog[:, 3]

            Tau_a = np.dot(np.cross(P_a[0:3] - P_e[0:3], measured_base_wrench_6D[0:3]), obj_pose_homog[0:3, 2])
            Tau_b = np.dot(np.cross(P_b[0:3] - P_e[0:3], measured_base_wrench_6D[0:3]), obj_pose_homog[0:3, 2])
            Tau_net = np.dot(measured_base_wrench_6D[3:6], obj_pose_homog[0:3, 2])

            epsilon1 = (Tau_net - Tau_b) / (Tau_a - Tau_b)
            epsilon1 = np.max([np.min([epsilon1, 1]), 0])

            epsilon2 = 1 - epsilon1

            P0 = epsilon1 * P_a + epsilon2 * P_b

        else:
            P0 = obj_vertices_world[:, height_indices[0]]

    return P0

def estimate_hand_COP(measured_contact_wrench_6D,hand_points,contact_pose_homog,l_contact):
    hand_points_world = np.dot(contact_pose_homog,hand_points)
    moment_arm_length = measured_contact_wrench_6D[-1] / measured_contact_wrench_6D[0]

    moment_arm_length = np.max([np.min([moment_arm_length, l_contact / 2]), -l_contact / 2])
    alpha_2 = (moment_arm_length + l_contact / 2) / (l_contact)
    alpha_1 = 1 - alpha_2
    return np.dot(hand_points, np.array([alpha_1, alpha_2])), np.dot(hand_points_world, np.array([alpha_1, alpha_2]))


def plot_force_arrow(cv_image,force_origin,force_vector,force_scale,camera_transformation_matrix):
    if force_origin is not None and force_vector is not None:
        force_tip = np.hstack([force_origin[0:3] + force_scale * force_vector,np.array([1])])
        x_coord, y_coord = get_pix_easier(
        np.vstack([force_origin, force_tip]).T,camera_transformation_matrix)
        cv2.arrowedLine(cv_image, (x_coord[0], y_coord[0]),(x_coord[1], y_coord[1]), (255, 0, 0),thickness=2)

def plot_hand_friction_cone(cv_image,COP_point_hand_frame,friction_parameter_dict,contact_pose_homog,camera_transformation_matrix,force_scale):
    if friction_parameter_dict is not None and COP_point_hand_frame is not None and contact_pose_homog is not None:
        P0_L = [
            friction_parameter_dict["acl"][0] * friction_parameter_dict["bcl"],
            friction_parameter_dict["acl"][1] * friction_parameter_dict["bcl"]
        ]
        P0_R = [
            friction_parameter_dict["acr"][0] * friction_parameter_dict["bcr"],
            friction_parameter_dict["acr"][1] * friction_parameter_dict["bcr"]
        ]

        x_left0 = P0_L[0] - 40 * force_scale * friction_parameter_dict["acl"][1]
        x_left1 = P0_L[0] + 0 * force_scale * friction_parameter_dict["acl"][1]
        y_left0 = P0_L[1] + 40 * force_scale * friction_parameter_dict["acl"][0]
        y_left1 = P0_L[1] - 0 * force_scale * friction_parameter_dict["acl"][0]

        left_bound0 = COP_point_hand_frame + np.array([x_left0, y_left0, 0.0, 0.0])
        left_bound1 = COP_point_hand_frame + np.array([x_left1, y_left1, 0.0, 0.0])

        left_boundary_list = np.vstack([left_bound0,left_bound1]).T

        x_coord, y_coord = get_pix_easier(np.dot(contact_pose_homog, left_boundary_list),camera_transformation_matrix)
        cv2.polylines(cv_image, [np.vstack([x_coord, y_coord]).T],True, (0, 255, 0),thickness=2)

        x_right0 = P0_R[0] - 0 * force_scale * friction_parameter_dict["acr"][1]
        x_right1 = P0_R[0] + 40 * force_scale * friction_parameter_dict["acr"][1]
        y_right0 = P0_R[1] + 0 * force_scale * friction_parameter_dict["acr"][0]
        y_right1 = P0_R[1] - 40 * force_scale * friction_parameter_dict["acr"][0]

        right_bound0 = COP_point_hand_frame + np.array([x_right0, y_right0, 0.0, 0.0])
        right_bound1 = COP_point_hand_frame + np.array([x_right1, y_right1, 0.0, 0.0])

        right_boundary_list = np.vstack([right_bound0, right_bound1]).T

        x_coord, y_coord = get_pix_easier(np.dot(contact_pose_homog, right_boundary_list),camera_transformation_matrix)
        cv2.polylines(cv_image, [np.vstack([x_coord, y_coord]).T],True, (0, 255, 0),thickness=2)

def plot_ground_friction_cone(cv_image,COP_ground,friction_parameter_dict,camera_transformation_matrix,force_scale):
    if friction_parameter_dict is not None and COP_ground is not None:    
        theta_list = []
        for A_vector in friction_parameter_dict["aer"]:
            theta_list.append(
                np.arctan2(A_vector[1], A_vector[0]))
        for A_vector in friction_parameter_dict["ael"]:
            theta_list.append(
                np.arctan2(A_vector[1], A_vector[0]))
        theta_list.append(3 * np.pi / 2)
        B_list = np.hstack(
            [friction_parameter_dict["ber"], friction_parameter_dict["bel"], 40.0])
        theta_list = np.array(theta_list)
        theta_index_list = np.argsort(theta_list)

        theta_list = theta_list[theta_index_list]
        B_list = B_list[theta_index_list]

        myHullVertexListX, myHullVertexListY = enumerate_vertices_of_constraint_polygon(
            theta_list, B_list)

        myHullVertexListX = -force_scale * np.array(myHullVertexListX) + COP_ground[0]
        myHullVertexListY = -force_scale * np.array(myHullVertexListY) + COP_ground[2]

        numPts = len(myHullVertexListX)
        BoundaryPts = np.vstack([
            myHullVertexListX,
            COP_ground[1] * np.ones([1, numPts]),
            myHullVertexListY,
            np.ones([1, numPts])
        ])

        x_coord, y_coord = get_pix_easier(BoundaryPts, camera_transformation_matrix)

        cv2.polylines(cv_image,[np.vstack([x_coord, y_coord]).T],False, (0, 255, 0),thickness=2)

def shape_overlay(cv_image,obj_pose_homog,object_vertex_array,camera_transformation_matrix):
    vertex_positions_world = np.dot(obj_pose_homog,object_vertex_array)
    x_coord, y_coord = get_pix_easier(vertex_positions_world, camera_transformation_matrix)
    cv2.polylines(cv_image,
                  [np.vstack([x_coord, y_coord]).T],
                  True, (0, 255, 0),
                  thickness=2)

def plot_ground_slide_arrow(cv_image,qp_debug_dict,hand_front_center_world,P0,camera_transformation_matrix,obj_pts=None,obj_pts_given=False):
    if qp_debug_dict is not None and 'err_x_pivot' in qp_debug_dict[
            'error_dict'] and (P0 is not None):

        dx_pivot = qp_debug_dict['error_dict']['err_x_pivot']

        if obj_pts_given:
            my_centroid = np.mean(obj_pts, axis=1)
        else:
            my_centroid = .5 * (P0 + hand_front_center_world)
            my_centroid[0] = hand_front_center_world[0]

        if np.abs(dx_pivot) > .002:
            Arrow_Start = my_centroid + np.array(
                [.05 * np.sign(dx_pivot), 0, 0, 0])
            Arrow_End = my_centroid + np.array(
                [.1 * np.sign(dx_pivot), 0, 0, 0])
            x_coord, y_coord = get_pix_easier(np.vstack([Arrow_Start, Arrow_End]).T,camera_transformation_matrix)
            cv2.arrowedLine(cv_image, (x_coord[0], y_coord[0]),(x_coord[1], y_coord[1]), (0, 0, 255), thickness=2)

def plot_pivot_arrow(cv_image,qp_debug_dict,hand_front_center_world,P0,camera_transformation_matrix):

    my_num_pts = 10
    if (qp_debug_dict is not None and 'err_theta' in qp_debug_dict['error_dict']
        and 'err_s' not in qp_debug_dict['error_dict'] and 'err_x_pivot' not in qp_debug_dict['error_dict']) and (P0 is not None):

        d_theta = qp_debug_dict['error_dict']['err_theta']
        if np.abs(d_theta) > np.pi / 85:
            R = .6 * np.linalg.norm(
                np.array([
                    hand_front_center_world[2] - P0[2],
                    hand_front_center_world[0] - P0[0]
                ]))
            my_angle = np.arctan2(
                hand_front_center_world[2] - P0[2],
                hand_front_center_world[0] - P0[0])

            my_theta_range = np.linspace(-np.sign(d_theta) * np.pi / 6,-.4 * np.sign(d_theta) * np.pi,num=my_num_pts) + my_angle
            X_pts = P0[0] + R * np.cos(my_theta_range)
            Y_pts = P0[2] + R * np.sin(my_theta_range)
            my_plot_pts = np.vstack([X_pts, P0[1] * np.ones([1, my_num_pts]),Y_pts,np.ones([1, my_num_pts])])

            x_coord, y_coord = get_pix_easier(my_plot_pts,camera_transformation_matrix)

            cv2.polylines(cv_image, 
                [np.vstack([x_coord[0:(my_num_pts - 1)],y_coord[0:(my_num_pts - 1)]]).T],
                False, (0, 0, 255),thickness=2)

            cv2.arrowedLine(cv_image, (x_coord[-2], y_coord[-2]),(x_coord[-1], y_coord[-1]), (0, 0, 255),thickness=2,tipLength=1)

def plot_hand_slide_arrow(cv_image,qp_debug_dict,hand_points,contact_pose_homog,camera_transformation_matrix):
    if qp_debug_dict is not None and 'err_s' in qp_debug_dict[
            'error_dict']:
        ds_hand = qp_debug_dict['error_dict']['err_s']
        desired_hand_points = hand_points + 0.0
        desired_hand_points[1, :] += ds_hand

        my_centroid = np.mean(hand_points, axis=1)

        if np.abs(ds_hand) > .002:
            Arrow_Start = my_centroid + np.array([0, .06 * np.sign(ds_hand), 0, 0])
            Arrow_End = my_centroid + np.array([0, .11 * np.sign(ds_hand), 0, 0])
            x_coord, y_coord = get_pix_easier(np.dot(contact_pose_homog,np.vstack([Arrow_Start, Arrow_End]).T),camera_transformation_matrix)
            cv2.arrowedLine(cv_image, (x_coord[0], y_coord[0]),(x_coord[1], y_coord[1]), (0, 0, 255),thickness=2)

def plot_impedance_target(cv_image,hand_points,target_pose_homog,camera_transformation_matrix):
    if target_pose_homog is not None:
        x_coord, y_coord = get_pix_easier(np.dot(target_pose_homog, hand_points),camera_transformation_matrix)
        cv2.polylines(cv_image, [np.vstack([x_coord, y_coord]).T],True, (0, 128, 255),thickness=2)

def plot_desired_object_pose(cv_image,qp_debug_dict,object_vertex_array,obj_pose_homog, camera_transformation_matrix):
    obj_vertices_world = np.dot(obj_pose_homog,object_vertex_array)
    obj_pts_desired = object_vertex_array

    if qp_debug_dict is not None and 'err_theta' in qp_debug_dict['error_dict']:
        d_theta = qp_debug_dict['error_dict']['err_theta']

        temp_vertex_array = object_vertex_array + 0.0

        height_indices = np.argsort(obj_vertices_world[2])
        min_vertex = object_vertex_array[0:3, height_indices[0]]
        for i in range(0, len(temp_vertex_array[0])):
            temp_vertex_array[0:3, i] -= min_vertex

        my_rotation = np.array([
            [np.cos(d_theta), -np.sin(d_theta), 0., min_vertex[0]],
            [np.sin(d_theta),
             np.cos(d_theta), 0., min_vertex[1]],
            [0., 0., 0., min_vertex[2]],
            [0., 0., 0., 1.],
        ])
        obj_pts_desired = np.dot(
            obj_pose_homog,
            np.dot(my_rotation, temp_vertex_array))

    if qp_debug_dict is not None and 'err_x_pivot' in qp_debug_dict['error_dict']:
        obj_pts_desired[0, :] += qp_debug_dict['error_dict']['err_x_pivot']

    x_coord, y_coord = get_pix_easier(obj_pts_desired,camera_transformation_matrix)

    cv2.polylines(cv_image,[np.vstack([x_coord, y_coord]).T],True, (0, 255, 0),thickness=2)

def compute_torque_bounds(hand_front_center_world,hand_normal,hand_tangent,contact_pose_homog,obj_vertices_world):
    hand_tangent_world      = np.dot(contact_pose_homog, hand_tangent)
    hand_normal_world       = np.dot(contact_pose_homog, hand_normal)

    hand_normal_world_offset = np.dot(np.transpose(hand_normal_world), hand_front_center_world)
    hand_tangent_world_offset = np.dot(np.transpose(hand_tangent_world), hand_front_center_world)

    hand_distance_list = np.abs(np.dot(np.transpose(hand_normal_world),obj_vertices_world) - hand_normal_world_offset)
    hand_tangent_list = np.dot(np.transpose(hand_tangent_world),obj_vertices_world) - hand_tangent_world_offset

    hand_distance_list = hand_distance_list[0]
    hand_tangent_list = hand_tangent_list[0]

    hand_distance_indices = np.argsort(hand_distance_list)
    new_tangent_list = np.sort([hand_tangent_list[hand_distance_indices[0]],hand_tangent_list[hand_distance_indices[1]]])

    left_bound = np.max([-l_contact / 2, new_tangent_list[0]])
    right_bound = np.min([l_contact / 2, new_tangent_list[1]])
    return np.array([left_bound, right_bound])


def synchronize_messages(data_dict):
    image_key = 'far_cam/color/image_raw'

    new_data_dict = {}
    index_list = {}
    time_list = {}
    index_list = {}
    for message_type in data_dict.keys():
        if message_type is not image_key and len(data_dict[message_type])>0:
            time_list[message_type] = data_dict[message_type][0]['time']
            new_data_dict[message_type] = []
            index_list[message_type] = 0
    
    for i in range(len(data_dict[image_key])):
        current_time = data_dict[image_key][i]['time']

        for message_type in new_data_dict.keys():
            while (index_list[message_type]<len(data_dict[message_type])-1 and 
                   data_dict[message_type][index_list[message_type]+1]['time']<=current_time):
                index_list[message_type]+=1
            new_data_dict[message_type].append(copy.deepcopy(data_dict[message_type][index_list[message_type]]))

    for message_type in new_data_dict.keys():
        data_dict[message_type] = new_data_dict[message_type]

    return new_data_dict

def invert_RBT(RBT_in):
    RBT_out = copy.deepcopy(RBT_in)

    RBT_out[0:3,0:3]=np.transpose(RBT_out[0:3,0:3])
    RBT_out[0:3,3]=-np.dot(RBT_out[0:3,0:3],RBT_out[0:3,3])
    return RBT_out

if __name__ == '__main__':

    cam_to_display = 'far'
    april_tag_cam = 'far'


    # my_path = 'C:/Users/taylorott/Dropbox (MIT)/pbal_assets/Experiments/InitialEstimatorDataPlusQPDebug-Jan-2022/'
    # fname = '2022-01-21-17-16-17-experiment012-rectangle-no-mass.pickle'
    my_path = '/home/robot2/Documents/panda/data/rosbag_data/'
    # fname = '2022-02-21-16-17-19-dummy-test-01-experiment0001'
    # fname = '2022-02-23-20-41-33-triange_hand_slide-experiment0001'
    # fname = '2022-02-23-22-48-29-triangle_execution_video-experiment0001'
    # fname = '2022-02-24-00-06-18-pentagon_execution_video-experiment0001'
    # fname = '2022-02-24-01-01-22-square_execution_video-experiment0001'
    # fname = '2022-02-24-01-39-36-hexagon_execution_video-experiment0001'
    # fname = '2022-02-24-01-58-40-rectangle_execution_video-experiment0001'
    # fname = '2022-02-24-02-07-56-expanding_cones_video-experiment0001'
    # fname = '2022-02-24-02-11-53-pivot_estimation_video-experiment0001'
    # fname = '2022-02-24-02-25-01-cup_video-experiment0001'
    # fname = '2022-02-24-02-48-21-red_solo_cup_video-experiment0001'
    # fname = '2022-02-24-03-02-42-big_box_video-experiment0001'
    fname = '2022-02-24-03-13-28-small_box_video-experiment0001'
    rotation_point_offset = .1
    near_camera_to_world_homog = pose_list_to_matrix([0.4384905809876356, -0.6034918747046369, 0.16814379761039527, -0.7017200051658107, 0.0036552740117039946, -0.0034100343559764624, 0.7124352917898473])
    far_camera_to_world_homog = pose_list_to_matrix([0.4980355178911215, 0.6018168729441976, 0.1394853205379789, 0.0034037964297700574, -0.6987986850579011, 0.7152145472836245, 0.011703131422520828])
    
    robot_apriltag_in_link8_homog = pose_list_to_matrix([-.04207285348, .04207285348, .073, 0.0, 0.0, .38268343, .9238795325])

    robot_ee_in_link8_homog = np.transpose(np.array(
        [[0.70710678,-0.70710678,0,0],
        [0.70710678,0.70710678,0,0],
        [0,0,1,0],
        [0.00707107,-0.00707107,0.116,1]]))
    link8_in_ee_homog = invert_RBT(robot_ee_in_link8_homog)

    robot_apriltag_in_ee_frame_homog = np.dot(link8_in_ee_homog,robot_apriltag_in_link8_homog)



    near_cam_transformation_matrix = np.array(
        [[920.9903714153191, 636.1672897044152, 9.392394039491016, -21.503085558889392], 
        [3.274304460321054, 366.19277918893073, -908.1154414222754, 372.2525941402742], 
        [0.009994071065983151, 0.999835264050234, 0.015151346643987835, 0.5964625469056951]])

    far_cam_transformation_matrix = np.array(
        [[-930.2832819054827, -604.5320201665304, 33.97255161021443, 822.3930138096654], 
        [-24.038407776196475, -386.36655209865575, -914.0708433060383, 371.99335563330385], 
        [-0.011487376253107511, -0.9996616405060643, 0.023337623862425626, 0.6040791079280076]])

    if cam_to_display == 'far':
        camera_transformation_matrix = far_cam_transformation_matrix
    if cam_to_display == 'near':
        camera_transformation_matrix = near_cam_transformation_matrix

    if april_tag_cam == 'far':
        camera_to_world_homog = far_camera_to_world_homog
    if april_tag_cam == 'near':
        camera_to_world_homog = near_camera_to_world_homog


    with open(my_path + fname + '.pickle', 'rb') as handle:
        data_dict = pickle.load(handle)



    synched_data_dict = synchronize_messages(data_dict)

    object_vertex_array, apriltag_id, apriltag_pos = load_shape_data('triangle')



    if cam_to_display == april_tag_cam:
        object_vertex_array = np.vstack([
            object_vertex_array,
            np.zeros(len(object_vertex_array[0])),
            np.ones(len(object_vertex_array[0]))
        ])
    else:
        object_vertex_array = np.vstack([
            object_vertex_array,
            -.068*np.ones(len(object_vertex_array[0])),
            np.ones(len(object_vertex_array[0]))
        ])



    # tag_camera_frame_homog = pose_list_to_matrix(data_dict['tag_detections'][count]['msg']['position']
    #                                             +data_dict['tag_detections'][count]['msg']['orientation'])
    # april_tag_pose_marker_frame_homog =  
    marker_pose_april_tag_frame_homog =  pose_list_to_matrix([-apriltag_pos[0],-apriltag_pos[1], 0,0.0,0.0,0.0,1.0])
    april_tag_pose_marker_frame_homog =  pose_list_to_matrix([ apriltag_pos[0], apriltag_pos[1], 0,0.0,0.0,0.0,1.0])
    # vertex_array_marker_frame = np.dot(april_tag_pose_marker_frame_homog,object_vertex_array)




    hand_points = np.array([[0.0, 0.0], [.05, -.05],
                            [0.041, 0.041], [1.0, 1.0]])
    hand_tangent = np.array([[0.0], [1.0], [0.0], [0.0]])
    hand_normal = np.array([[1.0], [0.0], [0.0], [0.0]])
    hand_front_center = np.array([0.0, 0.0, .041, 1.0])
    rotation_point_hand = np.array([rotation_point_offset, 0.0, .041, 1.0])

    l_hand_tag = .042
    x_tag_boundary = np.array([-l_hand_tag/2,l_hand_tag/2,l_hand_tag/2,-l_hand_tag/2])
    y_tag_boundary = np.array([l_hand_tag/2,l_hand_tag/2,-l_hand_tag/2,-l_hand_tag/2])
    z_tag_boundary = np.array([0.0]*4)
    one_tag_boundary = np.array([1.0]*4)

    hand_tag_boundary_pts = np.vstack([x_tag_boundary,y_tag_boundary,z_tag_boundary,one_tag_boundary])


    force_scale = .001


    plot_estimated_pivot = True


    bridge = CvBridge()

    sys_params = SystemParams()
    l_contact = sys_params.object_params["L_CONTACT_MAX"]




    ee_pose_world = None
    measured_base_wrench_6D = None
    measured_contact_wrench_6D = None
    friction_parameter_dict = None
    tag_camera_frame_homog = None
    qp_debug_dict = None
    pivot_xyz_estimated = None
    P0_estimated = None
    img_array = []
    obj_pose_homog = None

    dt_overall = data_dict['far_cam/color/image_raw'][-1]['time']-data_dict['far_cam/color/image_raw'][0]['time']
    num_frames = len(data_dict['far_cam/color/image_raw'])
    my_fps = np.round((num_frames-1)/dt_overall)
  

    for count in range(len(data_dict['far_cam/color/image_raw'])):
        cv_image = data_dict['far_cam/color/image_raw'][count]['msg']

        height, width, layers = cv_image.shape
        size = (width,height)


        if 'ee_pose_in_world_from_franka_publisher' in synched_data_dict.keys():
            ee_pose_world =  data_dict['ee_pose_in_world_from_franka_publisher'][count]['msg']
            contact_pose_homog = pose_list_to_matrix(ee_pose_world)
            hand_front_center_world = np.dot(contact_pose_homog,hand_front_center)
            robot_apriltag_pose_matrix = np.dot(contact_pose_homog,robot_apriltag_in_ee_frame_homog)
            rotation_point_hand_world_frame = np.dot(contact_pose_homog,rotation_point_hand)
        if 'end_effector_sensor_in_base_frame' in synched_data_dict.keys(): 
            measured_base_wrench_6D = np.array(data_dict['end_effector_sensor_in_base_frame'][count]['msg'])

        if 'end_effector_sensor_in_end_effector_frame' in synched_data_dict.keys():
            measured_contact_wrench_6D = np.array(data_dict['end_effector_sensor_in_end_effector_frame'][count]['msg'])

        if 'friction_parameters' in synched_data_dict.keys():
            friction_parameter_dict = data_dict['friction_parameters'][count]['msg']

        if 'tag_detections' in synched_data_dict.keys() and data_dict['tag_detections'][count]['msg'] is not None:
            tag_camera_frame_homog = pose_list_to_matrix(data_dict['tag_detections'][count]['msg']['position']+data_dict['tag_detections'][count]['msg']['orientation'])
            obj_pose_homog = np.dot(camera_to_world_homog,np.dot(tag_camera_frame_homog,marker_pose_april_tag_frame_homog))
        else:
            obj_pose_homog = None

        if 'qp_debug_message' in synched_data_dict.keys():
            qp_debug_dict = data_dict['qp_debug_message'][count]['msg']

        if 'target_frame' in synched_data_dict.keys():
            target_pose_homog = pose_list_to_matrix(data_dict['target_frame'][count]['msg'])

        if 'pivot_frame_estimated' in synched_data_dict.keys():
            pivot_frame_estimated = data_dict['pivot_frame_estimated'][count]['msg']
            pivot_xyz_estimated = pivot_frame_estimated[0:3]

        if pivot_xyz_estimated is not None and hand_front_center_world is not None:
            P0_estimated = [pivot_xyz_estimated[0], hand_front_center_world[1],pivot_xyz_estimated[2], 1.0]
            

        # if target_pose_homog is not None:
        #     plot_impedance_target(cv_image,hand_points,target_pose_homog,camera_transformation_matrix)


        if plot_estimated_pivot and P0_estimated is not None:
            plot_ground_friction_cone(cv_image,P0_estimated,friction_parameter_dict,camera_transformation_matrix,force_scale)
            if measured_base_wrench_6D is not None:
                plot_force_arrow(cv_image,P0_estimated,measured_base_wrench_6D[0:3],force_scale,camera_transformation_matrix)
            plot_pivot_arrow(cv_image,qp_debug_dict,hand_front_center_world,P0_estimated,camera_transformation_matrix)
            plot_ground_slide_arrow(cv_image,qp_debug_dict,hand_front_center_world,P0_estimated,camera_transformation_matrix)
        else:
            plot_pivot_arrow(cv_image,qp_debug_dict,hand_front_center_world,rotation_point_hand_world_frame,camera_transformation_matrix)
            plot_ground_slide_arrow(cv_image,qp_debug_dict,hand_front_center_world,rotation_point_hand_world_frame,camera_transformation_matrix)
            

        if measured_contact_wrench_6D is not None and measured_base_wrench_6D is not None:
            if np.abs(measured_contact_wrench_6D[0]) > .1:
                hand_COP_hand_frame, hand_COP_world_frame = estimate_hand_COP(measured_contact_wrench_6D,hand_points,contact_pose_homog,l_contact)
                plot_hand_friction_cone(cv_image,hand_COP_hand_frame,friction_parameter_dict,contact_pose_homog,camera_transformation_matrix,force_scale)
                plot_force_arrow(cv_image,hand_COP_world_frame,-measured_base_wrench_6D[0:3],force_scale,camera_transformation_matrix)

        plot_hand_slide_arrow(cv_image,qp_debug_dict,hand_points,contact_pose_homog,camera_transformation_matrix)

        # shape_overlay(cv_image,robot_apriltag_pose_matrix,hand_tag_boundary_pts,camera_transformation_matrix)




        # if tag_camera_frame_homog is not None:
        #     # shape_overlay(cv_image,obj_pose_homog,object_vertex_array,camera_transformation_matrix)
        #     current_dot_positions = np.dot(obj_pose_homog,object_vertex_array)
        #     # plot_desired_object_pose(cv_image,qp_debug_dict,object_vertex_array,obj_pose_homog, camera_transformation_matrix)
        #     plot_ground_slide_arrow(cv_image,qp_debug_dict,hand_front_center_world,None,camera_transformation_matrix,current_dot_positions,True)
        #     if measured_contact_wrench_6D is not None and measured_base_wrench_6D is not None:
        #         P0 = estimate_ground_COP(current_dot_positions,measured_base_wrench_6D)
        #         if P0 is not None:
        #             plot_ground_friction_cone(cv_image,P0,friction_parameter_dict,camera_transformation_matrix,force_scale)
        #             plot_pivot_arrow(cv_image,qp_debug_dict,hand_front_center_world,P0,camera_transformation_matrix)
        #             plot_force_arrow(cv_image,P0,measured_base_wrench_6D[0:3],force_scale,camera_transformation_matrix)

        img_array.append(cv_image)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    video_out = cv2.VideoWriter(my_path + fname+'.avi',cv2.VideoWriter_fourcc(*'DIVX'), my_fps, size)

    for i in range(len(img_array)):
        video_out.write(img_array[i])
    video_out.release() 
 
  




