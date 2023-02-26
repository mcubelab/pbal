#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import copy
import cv2
from cvxopt import matrix, solvers

import numpy as np

def pose_list_to_matrix(pose_in):
    rotation_matrix = quat_to_mat(pose_in[3:7])
    translation_vector = np.array([pose_in[0:3]])
    return np.vstack([
        np.hstack([rotation_matrix, np.transpose(translation_vector)]),
        np.array([0.0, 0.0, 0.0, 1.0])])

# This function was copy-pasted from the Automatic Addison website
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

    return np.array([[r00, r01, r02],
                     [r10, r11, r12],
                     [r20, r21, r22]])

def load_shape_data(name_in):
    import json

    curr_dir = os.path.dirname(
        os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(curr_dir)
    gparentdir = os.path.dirname(parentdir)
    print('parentdir', parentdir)
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

    # build constraint matrix for the polygon
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

    # find the first vertex
    x_sol = np.squeeze(sol['x'])
    vertex_x_list.append(x_sol[0])
    vertex_y_list.append(x_sol[1])

    # active constraints for the first vertex
    dual_list = np.squeeze(sol['z'])
    dual_value_index_list = np.argsort(dual_list)
    # print np.sort(np.squeeze(sol['z']))

    if len(dual_value_index_list) < 3:
        return np.array([]), np.array([])

    # find the active constraint for the first vertex
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

def plot_pivot_dot(cv_image, pivot_location, camera_transformation_matrix,color = None, radius = None):
        if color is None:
            color = (0, 0, 255)

        if radius is None:
            radius = 5
        x_coord, y_coord = get_pix_easier(pivot_location.T, camera_transformation_matrix)
        cv2.circle(cv_image, (x_coord[0], y_coord[0]), radius , color, -1)

def estimate_ground_COP(obj_vertices_world_manipulation, measured_world_manipulation_wrench_6D, contact_pose_homog, obj_pose_homog = None, height_threshold=.01, vertices_in = None):
    P0 = None

    if measured_world_manipulation_wrench_6D is not None and obj_vertices_world_manipulation is not None:
        height_indices = np.argsort(obj_vertices_world_manipulation[0])
        if vertices_in is not None and len(vertices_in)==2:
            P_a = obj_vertices_world_manipulation[:, vertices_in[0]]
            P_b = obj_vertices_world_manipulation[:, vertices_in[1]]

            P_e = contact_pose_homog[:, 3]

            Tau_a = np.dot(np.cross(
                P_a[0:3] - P_e[0:3], measured_world_manipulation_wrench_6D[0:3]), np.array([0.0,0.0,1.0]))
            Tau_b = np.dot(np.cross(
                P_b[0:3] - P_e[0:3], measured_world_manipulation_wrench_6D[0:3]), np.array([0.0,0.0,1.0]))
            Tau_net = np.dot(
                measured_world_manipulation_wrench_6D[3:6], np.array([0.0,0.0,1.0]))

            epsilon1 = (Tau_net - Tau_b) / (Tau_a - Tau_b)
            epsilon1 = np.max([np.min([epsilon1, 1]), 0])

            epsilon2 = 1 - epsilon1

            P0 = epsilon1 * P_a + epsilon2 * P_b

        elif abs(obj_vertices_world_manipulation[0][height_indices[1]] - obj_vertices_world_manipulation[0][height_indices[0]]) < height_threshold:
            P_a = obj_vertices_world_manipulation[:, height_indices[0]]
            P_b = obj_vertices_world_manipulation[:, height_indices[1]]

            P_e = contact_pose_homog[:, 3]

            Tau_a = np.dot(np.cross(
                P_a[0:3] - P_e[0:3], measured_world_manipulation_wrench_6D[0:3]), np.array([0.0,0.0,1.0]))
            Tau_b = np.dot(np.cross(
                P_b[0:3] - P_e[0:3], measured_world_manipulation_wrench_6D[0:3]), np.array([0.0,0.0,1.0]))
            Tau_net = np.dot(
                measured_world_manipulation_wrench_6D[3:6], np.array([0.0,0.0,1.0]))

            epsilon1 = (Tau_net - Tau_b) / (Tau_a - Tau_b)
            epsilon1 = np.max([np.min([epsilon1, 1]), 0])

            epsilon2 = 1 - epsilon1

            P0 = epsilon1 * P_a + epsilon2 * P_b

        else:
            P0 = obj_vertices_world_manipulation[:, height_indices[0]]

    return P0


def estimate_hand_COP(measured_contact_wrench_6D, hand_points, contact_pose_homog, l_contact):
    hand_points_world = np.dot(contact_pose_homog, hand_points)
    moment_arm_length = measured_contact_wrench_6D[-1] / \
        measured_contact_wrench_6D[0]

    moment_arm_length = np.max(
        [np.min([moment_arm_length, l_contact / 2]), -l_contact / 2])
    alpha_2 = (moment_arm_length + l_contact / 2) / (l_contact)
    alpha_1 = 1 - alpha_2
    return np.dot(hand_points, np.array([alpha_1, alpha_2])), np.dot(hand_points_world, np.array([alpha_1, alpha_2]))




def plot_force_arrow(cv_image, force_origin, force_vector, force_scale, camera_transformation_matrix, thickness = None):
    if thickness is None:
        thickness = 2
    force_origin = np.hstack([force_origin[0:3],np.array([1])])
    if force_origin is not None and force_vector is not None:
        force_tip = np.hstack(
            [force_origin[0:3] + force_scale * force_vector, np.array([1])])
        x_coord, y_coord = get_pix_easier(
            np.vstack([force_origin, force_tip]).T, camera_transformation_matrix)
        cv2.arrowedLine(cv_image, (x_coord[0], y_coord[0]),
                        (x_coord[1], y_coord[1]), (255, 0, 0), thickness=thickness)


def plot_hand_friction_cone(cv_image, COP_point_hand_frame, friction_parameter_dict, contact_pose_homog, camera_transformation_matrix, force_scale, thickness = None):
    if thickness is None:
        thickness = 2
    if friction_parameter_dict is not None and COP_point_hand_frame is not None and contact_pose_homog is not None:
        P0_L = [
            friction_parameter_dict["acl"][0] * friction_parameter_dict["bcl"],
            friction_parameter_dict["acl"][1] * friction_parameter_dict["bcl"]
        ]
        P0_R = [
            friction_parameter_dict["acr"][0] * friction_parameter_dict["bcr"],
            friction_parameter_dict["acr"][1] * friction_parameter_dict["bcr"]
        ]

        x_left0 = P0_L[0] - 40 * force_scale * \
            friction_parameter_dict["acl"][1]
        x_left1 = P0_L[0] + 0 * force_scale * friction_parameter_dict["acl"][1]
        y_left0 = P0_L[1] + 40 * force_scale * \
            friction_parameter_dict["acl"][0]
        y_left1 = P0_L[1] - 0 * force_scale * friction_parameter_dict["acl"][0]

        left_bound0 = COP_point_hand_frame + \
            np.array([x_left0, y_left0, 0.0, 0.0])
        left_bound1 = COP_point_hand_frame + \
            np.array([x_left1, y_left1, 0.0, 0.0])

        left_boundary_list = np.vstack([left_bound0, left_bound1]).T

        x_coord, y_coord = get_pix_easier(
            np.dot(contact_pose_homog, left_boundary_list), camera_transformation_matrix)
        cv2.polylines(cv_image, [np.vstack(
            [x_coord, y_coord]).T], True, (0, 255, 0), thickness=thickness)

        x_right0 = P0_R[0] - 0 * force_scale * \
            friction_parameter_dict["acr"][1]
        x_right1 = P0_R[0] + 40 * force_scale * \
            friction_parameter_dict["acr"][1]
        y_right0 = P0_R[1] + 0 * force_scale * \
            friction_parameter_dict["acr"][0]
        y_right1 = P0_R[1] - 40 * force_scale * \
            friction_parameter_dict["acr"][0]

        right_bound0 = COP_point_hand_frame + \
            np.array([x_right0, y_right0, 0.0, 0.0])
        right_bound1 = COP_point_hand_frame + \
            np.array([x_right1, y_right1, 0.0, 0.0])

        right_boundary_list = np.vstack([right_bound0, right_bound1]).T

        x_coord, y_coord = get_pix_easier(
            np.dot(contact_pose_homog, right_boundary_list), camera_transformation_matrix)
        cv2.polylines(cv_image, [np.vstack(
            [x_coord, y_coord]).T], True, (0, 255, 0), thickness=thickness)



def plot_wm_lines(cv_image, wm_coords, camera_transformation_matrix,color = None, thickness = None):
        if color is None:
            color = (0, 0, 255)

        if thickness is None:
            thickness = 2

        x_coord, y_coord = get_pix_easier(wm_coords, camera_transformation_matrix)
        cv2.polylines(cv_image, [np.vstack([x_coord, y_coord]).T], False, color, thickness=thickness)



def plot_ground_friction_cone(cv_image, COP_ground, friction_parameter_dict, camera_transformation_matrix, force_scale, thickness = None):
    if thickness is None:
        thickness = 2
    if friction_parameter_dict is not None and COP_ground is not None:
        theta_list = []
        for A_vector in friction_parameter_dict["aer"]:
            theta_list.append(
                np.arctan2(A_vector[1], A_vector[0]))
        for A_vector in friction_parameter_dict["ael"]:
            theta_list.append(
                np.arctan2(A_vector[1], A_vector[0]))
        theta_list.append(np.pi)
        B_list = np.hstack(
            [friction_parameter_dict["ber"], friction_parameter_dict["bel"], 40.0])
        theta_list = np.array(theta_list)
        theta_index_list = np.argsort(theta_list)

        theta_list = theta_list[theta_index_list]
        B_list = B_list[theta_index_list]

        myHullVertexListX, myHullVertexListY = enumerate_vertices_of_constraint_polygon(
            theta_list, B_list)

        myHullVertexListX = -force_scale * \
            np.array(myHullVertexListX) + COP_ground[0]
        myHullVertexListY = -force_scale * \
            np.array(myHullVertexListY) + COP_ground[1]

        numPts = len(myHullVertexListX)
        BoundaryPts = np.vstack([
            myHullVertexListX,
            myHullVertexListY,
            COP_ground[2] * np.ones([1, numPts]),
            np.ones([1, numPts])
        ])

        x_coord, y_coord = get_pix_easier(
            BoundaryPts, camera_transformation_matrix)

        cv2.polylines(cv_image, [np.vstack(
            [x_coord, y_coord]).T], False, (0, 255, 0), thickness=thickness)

def overlay_qp_ground_constraints(cv_image,COP_ground,friction_parameter_dict,contact_pose_homog,camera_transformation_matrix,force_scale,qp_debug_dict):
    ground_frame_homog = copy.deepcopy(contact_pose_homog)

    ground_frame_homog[0:3,3]=COP_ground[0:3]

    theta_list = []
    B_list = []
    for i in range(len(qp_debug_dict['constraint_offsets'])):
        if qp_debug_dict['label_list_cnstr'][i]=='fer' or qp_debug_dict['label_list_cnstr'][i]=='fel':
            A_vector = qp_debug_dict['constraint_normals'][i]
            b = qp_debug_dict['constraint_offsets'][i]
            theta_list.append(np.arctan2(A_vector[1], A_vector[0]))
            B_list.append(b)
        if qp_debug_dict['label_list_cnstr'][i]=='ncmx':
            b_max = qp_debug_dict['constraint_offsets'][i]
 

    my_transform = np.transpose(ground_frame_homog[0:3,0:3])
    vec_up = np.array([my_transform[0,2],my_transform[1,2]])
    vec_left = np.array([my_transform[0,0],my_transform[1,0]])
    vec_down = -vec_up
    vec_right = -vec_left


    vec_list = [vec_up,vec_down,vec_left,vec_right]
    b_max_list = [0.0,1.2*b_max,.7*b_max,.7*b_max]

    for i in range(len(vec_list)):
        A_vector = vec_list[i]
        b = b_max_list[i]
        theta_list.append(np.arctan2(A_vector[1], A_vector[0]))
        B_list.append(b)

    theta_list = np.array(theta_list)
    B_list = np.array(B_list)
    theta_index_list = np.argsort(theta_list)

    theta_list = theta_list[theta_index_list]
    B_list = B_list[theta_index_list]


    myHullVertexListX, myHullVertexListY = enumerate_vertices_of_constraint_polygon(
        theta_list, B_list)

    numPts = len(myHullVertexListX)


    myHullVertexListX = -force_scale * np.array(myHullVertexListX)
    myHullVertexListY = -force_scale * np.array(myHullVertexListY)


    numPts = len(myHullVertexListX)
    BoundaryPts = np.vstack([
        myHullVertexListX,
        myHullVertexListY,
        np.zeros([1, numPts]),
        np.ones([1, numPts])
    ])

    x_coord, y_coord = get_pix_easier(np.dot(ground_frame_homog,BoundaryPts), camera_transformation_matrix)
    
    if len(myHullVertexListX)>=3:
        cv2.fillPoly(cv_image,[np.vstack([x_coord, y_coord]).T],(0, 0, 255))

def overlay_qp_hand_constraints(cv_image,COP_point_hand_frame,hand_front_center,friction_parameter_dict,contact_pose_homog,camera_transformation_matrix,force_scale,qp_debug_dict):


    measured_wrench = qp_debug_dict['measured_wrench']

    for i in range(len(qp_debug_dict['constraint_offsets'])):
        if qp_debug_dict['label_list_cnstr'][i]=='flc' and qp_debug_dict['constraint_normals'][i][2]==-1.:
            # print('correction!')
            qp_debug_dict['label_list_cnstr'][i]='tlc'

    frc_on = False
    flc_on = False
    nmax_on = False

    if 'frc' in qp_debug_dict['label_list_cnstr']:
        frc_on = True
        frc_index = qp_debug_dict['label_list_cnstr'].index('frc')
        A_right = qp_debug_dict['constraint_normals'][frc_index]
        b_right = qp_debug_dict['constraint_offsets'][frc_index]
        # print('A_right', A_right)
    if 'flc' in qp_debug_dict['label_list_cnstr']:
        flc_on = True
        flc_index = qp_debug_dict['label_list_cnstr'].index('flc')
        A_left = qp_debug_dict['constraint_normals'][flc_index]
        b_left = qp_debug_dict['constraint_offsets'][flc_index]
        # print('A_left', A_left)
    if 'ncmx' in qp_debug_dict['label_list_cnstr']:
        nmax_on = True
        nmax_index = qp_debug_dict['label_list_cnstr'].index('ncmx')
        A_max = qp_debug_dict['constraint_normals'][nmax_index]
        b_max = qp_debug_dict['constraint_offsets'][nmax_index]
        # print('A_max', A_max)

        A_min = [1.0,0.0,0.0]
        b_min = 0.0

    trc_index = qp_debug_dict['label_list_cnstr'].index('trc')
    l_torque = qp_debug_dict['constraint_normals'][trc_index][0]


    torque_boundary_pts = np.transpose(np.vstack([hand_front_center,hand_front_center,hand_front_center,hand_front_center]))

    torque_boundary_pts[1,0] = l_torque
    torque_boundary_pts[1,1] = -l_torque
    torque_boundary_pts[1,2] = -l_torque
    torque_boundary_pts[1,3] = l_torque

    torque_boundary_pts[0,0] = .002
    torque_boundary_pts[0,1] = .002
    torque_boundary_pts[0,2] = -.002
    torque_boundary_pts[0,3] = -.002 

    if flc_on:
        top_left_vertex = np.linalg.solve(np.array([A_max[0:2],A_left[0:2]]),np.array([b_max,b_left]))

    if frc_on:
        top_right_vertex = np.linalg.solve(np.array([A_right[0:2],A_max[0:2]]),np.array([b_right,b_max]))   

    if frc_on and flc_on:
        bottom_vertex = np.linalg.solve(np.array([A_right[0:2],A_left[0:2]]),np.array([b_right,b_left]))

        x_vertex_hand_list = np.array([bottom_vertex[0],top_right_vertex[0],top_left_vertex[0]])
        y_vertex_hand_list = np.array([bottom_vertex[1],top_right_vertex[1],top_left_vertex[1]])

        numPts = 3 
  
    if not flc_on:
        bottom_vertex = np.linalg.solve(np.array([A_right[0:2],A_min[0:2]]),np.array([b_right,b_min]))

        numPts = 4

        x_vertex_hand_list = np.array([bottom_vertex[0],top_right_vertex[0],b_max,b_min])
        y_vertex_hand_list = np.array([bottom_vertex[1],top_right_vertex[1],-.7*np.sign(top_right_vertex[1])*b_max,-.7*np.sign(top_right_vertex[1])*b_max])

    if not frc_on:
        bottom_vertex = np.linalg.solve(np.array([A_min[0:2],A_left[0:2]]),np.array([b_min,b_left]))

        numPts = 4

        x_vertex_hand_list = np.array([bottom_vertex[0],top_left_vertex[0],b_max,b_min])
        y_vertex_hand_list = np.array([bottom_vertex[1],top_left_vertex[1],-.7*np.sign(top_left_vertex[1])*b_max,-.7*np.sign(top_left_vertex[1])*b_max])


    myHullVertexListX = force_scale * x_vertex_hand_list + COP_point_hand_frame[0]
    myHullVertexListY = force_scale * y_vertex_hand_list + COP_point_hand_frame[1]

    BoundaryPts = np.vstack([
        myHullVertexListX,
        myHullVertexListY,
        COP_point_hand_frame[2] * np.ones([1, numPts]),
        np.ones([1, numPts])
    ])

    x_coord, y_coord = get_pix_easier(np.dot(contact_pose_homog,BoundaryPts), camera_transformation_matrix)

    cv2.fillPoly(cv_image,[np.vstack([x_coord, y_coord]).T],(0, 0, 255))

    x_coord, y_coord = get_pix_easier(np.dot(contact_pose_homog,torque_boundary_pts), camera_transformation_matrix)
    cv2.fillPoly(cv_image,[np.vstack([x_coord, y_coord]).T],(80, 127, 255))
    # cv2.polylines(cv_image, [np.vstack([x_coord, y_coord]).T],True, (80, 127, 255),thickness=2)

def shape_overlay(cv_image, obj_pose_homog, object_vertex_array, camera_transformation_matrix,isclosed = True):
    vertex_positions_world = np.dot(obj_pose_homog, object_vertex_array)
    x_coord, y_coord = get_pix_easier(
        vertex_positions_world, camera_transformation_matrix)
    cv2.polylines(cv_image,
                  [np.vstack([x_coord, y_coord]).T],
                  isclosed, (0, 255, 0),
                  thickness=2)


def scatter_dots(cv_image, obj_pose_homog, object_vertex_array, camera_transformation_matrix):
    vertex_positions_world = np.dot(obj_pose_homog, object_vertex_array)
    x_coord, y_coord = get_pix_easier(
        vertex_positions_world, camera_transformation_matrix)

    for i in range(len(x_coord)):
        cv2.circle(cv_image, (x_coord[i], y_coord[i]), 2 , (0, 255, 0), -1)


def plot_ground_slide_arrow(cv_image, qp_debug_dict, hand_front_center_world, P0, camera_transformation_matrix, obj_pts=None, obj_pts_given=False):
    if qp_debug_dict is not None and 'error_s_pivot' in qp_debug_dict[
            'error_dict'] and (P0 is not None):

        dx_pivot = -qp_debug_dict['error_dict']['error_s_pivot']

        if obj_pts_given:
            my_centroid = np.mean(obj_pts, axis=1)
        else:
            my_centroid = .5 * (P0 + hand_front_center_world)
            my_centroid[0] = hand_front_center_world[0]

        if np.abs(dx_pivot) > .002:
            Arrow_Start = my_centroid + np.array(
                [0, .05 * np.sign(dx_pivot), 0, 0])
            Arrow_End = my_centroid + np.array(
                [0, .1 * np.sign(dx_pivot), 0, 0])
            x_coord, y_coord = get_pix_easier(
                np.vstack([Arrow_Start, Arrow_End]).T, camera_transformation_matrix)
            cv2.arrowedLine(
                cv_image, (x_coord[0], y_coord[0]), (x_coord[1], y_coord[1]), (0, 0, 255), thickness=2)


def plot_pivot_arrow(cv_image, qp_debug_dict, hand_front_center_world, P0, camera_transformation_matrix):

    my_num_pts = 10
    if (qp_debug_dict is not None and 'error_theta' in qp_debug_dict['error_dict']
            and 'error_s_pivot' not in qp_debug_dict['error_dict'] and 'error_s_hand' not in qp_debug_dict['error_dict']) and (P0 is not None):

        d_theta = qp_debug_dict['error_dict']['error_theta']
        if np.abs(d_theta) > np.pi / 85:
            R = .6 * np.linalg.norm(
                np.array([
                    hand_front_center_world[2] - P0[2],
                    hand_front_center_world[0] - P0[0]
                ]))
            my_angle = np.arctan2(
                hand_front_center_world[2] - P0[2],
                hand_front_center_world[0] - P0[0])

            my_theta_range = np.linspace(-np.sign(d_theta) * np.pi /
                                         6, -.4 * np.sign(d_theta) * np.pi, num=my_num_pts) + my_angle
            X_pts = P0[0] + R * np.cos(my_theta_range)
            Y_pts = P0[1] + R * np.sin(my_theta_range)
            my_plot_pts = np.vstack(
                [X_pts, Y_pts, P0[2] * np.ones([1, my_num_pts]), np.ones([1, my_num_pts])])

            x_coord, y_coord = get_pix_easier(
                my_plot_pts, camera_transformation_matrix)

            cv2.polylines(cv_image,
                          [np.vstack([x_coord[0:(my_num_pts - 1)],
                                     y_coord[0:(my_num_pts - 1)]]).T],
                          False, (0, 0, 255), thickness=2)

            cv2.arrowedLine(cv_image, (x_coord[-2], y_coord[-2]), (x_coord[-1],
                            y_coord[-1]), (0, 0, 255), thickness=2, tipLength=1)


def plot_hand_slide_arrow(cv_image, qp_debug_dict, hand_points, contact_pose_homog, camera_transformation_matrix):
    if qp_debug_dict is not None and 'error_s_hand' in qp_debug_dict[
            'error_dict']:
        ds_hand = -qp_debug_dict['error_dict']['error_s_hand']
        desired_hand_points = hand_points + 0.0
        desired_hand_points[1, :] += ds_hand

        my_centroid = np.mean(hand_points, axis=1)

        if np.abs(ds_hand) > .002:
            Arrow_Start = my_centroid + \
                np.array([0, .06 * np.sign(ds_hand), 0, 0])
            Arrow_End = my_centroid + \
                np.array([0, .11 * np.sign(ds_hand), 0, 0])
            x_coord, y_coord = get_pix_easier(np.dot(contact_pose_homog, np.vstack(
                [Arrow_Start, Arrow_End]).T), camera_transformation_matrix)
            cv2.arrowedLine(
                cv_image, (x_coord[0], y_coord[0]), (x_coord[1], y_coord[1]), (0, 0, 255), thickness=2)


def plot_impedance_target(cv_image, hand_points, target_pose_homog, camera_transformation_matrix):
    if target_pose_homog is not None:
        x_coord, y_coord = get_pix_easier(
            np.dot(target_pose_homog, hand_points), camera_transformation_matrix)
        cv2.polylines(cv_image, [np.vstack(
            [x_coord, y_coord]).T], True, (0, 128, 255), thickness=2)


def plot_desired_object_pose(cv_image, qp_debug_dict, object_vertex_array, obj_pose_homog, camera_transformation_matrix):
    obj_vertices_world = np.dot(obj_pose_homog, object_vertex_array)
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

    x_coord, y_coord = get_pix_easier(
        obj_pts_desired, camera_transformation_matrix)

    cv2.polylines(cv_image, [np.vstack(
        [x_coord, y_coord]).T], True, (0, 255, 0), thickness=2)


def compute_torque_bounds(hand_front_center_world, hand_normal, hand_tangent, contact_pose_homog, obj_vertices_world):
    hand_tangent_world = np.dot(contact_pose_homog, hand_tangent)
    hand_normal_world = np.dot(contact_pose_homog, hand_normal)

    hand_normal_world_offset = np.dot(np.transpose(
        hand_normal_world), hand_front_center_world)
    hand_tangent_world_offset = np.dot(np.transpose(
        hand_tangent_world), hand_front_center_world)

    hand_distance_list = np.abs(np.dot(np.transpose(
        hand_normal_world), obj_vertices_world) - hand_normal_world_offset)
    hand_tangent_list = np.dot(np.transpose(
        hand_tangent_world), obj_vertices_world) - hand_tangent_world_offset

    hand_distance_list = hand_distance_list[0]
    hand_tangent_list = hand_tangent_list[0]

    hand_distance_indices = np.argsort(hand_distance_list)
    new_tangent_list = np.sort(
        [hand_tangent_list[hand_distance_indices[0]], hand_tangent_list[hand_distance_indices[1]]])

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
        if message_type is not image_key and len(data_dict[message_type]) > 0:
            time_list[message_type] = data_dict[message_type][0]['time']
            new_data_dict[message_type] = []
            index_list[message_type] = 0

    for i in range(len(data_dict[image_key])):
        current_time = data_dict[image_key][i]['time']

        for message_type in new_data_dict.keys():
            while (index_list[message_type] < len(data_dict[message_type])-1 and
                   data_dict[message_type][index_list[message_type]+1]['time'] <= current_time):
                index_list[message_type] += 1
            new_data_dict[message_type].append(copy.deepcopy(
                data_dict[message_type][index_list[message_type]]))

    for message_type in new_data_dict.keys():
        data_dict[message_type] = new_data_dict[message_type]

    return new_data_dict


def invert_RBT(RBT_in):
    RBT_out = copy.deepcopy(RBT_in)

    RBT_out[0:3, 0:3] = np.transpose(RBT_out[0:3, 0:3])
    RBT_out[0:3, 3] = -np.dot(RBT_out[0:3, 0:3], RBT_out[0:3, 3])
    return RBT_out