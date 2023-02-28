#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))


from Modelling.system_params import SystemParams
from Helpers.ros_manager import ros_manager
from Estimation.tests import test_gtsam_advanced_estimator
from Estimation import fast_vision_polygon_estimator_node
from Helpers.camera_transform_manager import camera_transform_manager
import cv2

import Helpers.pbal_msg_helper as pmh
import pickle
import numpy as np

subscribe_required_list = ['/torque_cone_boundary_test',
                          '/torque_cone_boundary_flag',
                          '/sliding_state',
                          '/ee_pose_in_world_manipulation_from_franka_publisher',
                          '/end_effector_sensor_in_world_manipulation_frame',
                          '/end_effector_sensor_in_end_effector_frame',
                          '/ee_pose_in_base_from_franka_publisher',
                          '/friction_parameters',
                          '/near_cam/color/camera_info',
                          '/near_cam/color/image_raw']

subscribe_not_required_list = [   '/netft/netft_data',
                                  '/pivot_frame_realsense',
                                  '/generalized_positions',
                                  '/barrier_func_control_command',
                                  '/torque_bound_message',
                                  '/qp_debug_message',
                                  '/target_frame',
                                  '/pivot_sliding_commanded_flag',
                                  '/polygon_contact_estimate',
                                  '/polygon_vision_estimate',
                                  '/sliding_state',
                                  '/torque_cone_boundary_flag',
                                  '/torque_cone_boundary_test',
                                  '/pivot_frame_estimated']

def update_pickle_with_fast_vision(fname_in,read_path, frame_rate):

    rm = ros_manager(load_mode = True, path=read_path, fname=fname_in)

    rm.setRate(frame_rate)
   
    rm.spawn_transform_listener()

    rm.subscribe_to_list(subscribe_required_list,True)
    rm.subscribe_to_list(subscribe_not_required_list,False)

    rm.wait_for_necessary_data()

    my_fast_vision_generator = fast_vision_polygon_estimator_node.polygon_vision_operator(rm)

    msg_list = []
    time_list = []

    while rm.read_still_running():
        rm.unpack_all()
        my_fast_vision_generator.update_estimator()

        if my_fast_vision_generator.can_publish:
            msg = pmh.parse_polygon_contact_state_stamped(pmh.generate_polygon_contact_state_stamped(my_fast_vision_generator.vertex_array_to_publish,[]))
            msg_list.append(msg)
            time_list.append(rm.eval_current_time())

        rm.sleep()

    read_dict = None
    with open(read_path+fname_in, 'rb') as handle:
        read_dict = pickle.load(handle)
    handle.close()

    read_dict['/polygon_vision_estimate'] = {'time_list':time_list,'msg_list':msg_list}

    with open(read_path+fname_in, 'wb') as handle:
        pickle.dump(read_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)
    handle.close()


def compute_error(v0,v1):
    num_vertices = len(v0[0])

    if len(v0[0])==len(v1[0]):
        total_squared_error = 0.0
        for i in range(num_vertices):
            min_error = None
            for j in range(num_vertices):
                vertex_error = np.linalg.norm(v0[:,i]-v1[:,j])


                if min_error is None or vertex_error<min_error:
                    min_error = vertex_error

            if min_error>.05:

                print('uh oh!')
                print(v0)
                print(v1)
            total_squared_error+=min_error**2

        return total_squared_error
    else:
        return None

def compute_error2(ref_vertex,v1):
    num_vertices = len(v1[0])

    min_error = None
    for i in range(num_vertices):
        vertex_error = np.linalg.norm(ref_vertex-v1[:,i])


        if min_error is None or vertex_error<min_error:
            min_error = vertex_error

    if min_error>.05:

        print('uh oh!')
        print(ref_vertex)
        print(v1)

    return min_error**2


def april_tag_comparison(fname_in,read_path, frame_rate):
    rm = ros_manager(load_mode = True, path=read_path, fname=fname_in)

    rm.setRate(frame_rate)
    rm.spawn_transform_listener()
    rm.subscribe_to_list(subscribe_required_list,True)
    rm.subscribe_to_list(subscribe_not_required_list,False)


    shape_name_list = ['big_triangle','big_square','rectangle','square','triangle','big_rectangle','rectangle_bump_in','rectangle_bump_out']

    cam_choice = 'near'

    ctm = camera_transform_manager(rm,cam_choice)
    ctm.setup_frames()
    camera_transformation_matrix = ctm.generate_camera_transformation_matrix()
    shape_dict = ctm.load_shape_data(shape_name_list)

    rm.wait_for_necessary_data()


    object_vertex_array = test_gtsam_advanced_estimator.get_shape_prior(load_mode = True, path = read_path, fname = fname_in)
    my_fast_vision_generator = fast_vision_polygon_estimator_node.polygon_vision_operator(rm)
    my_estimator_with_vision = test_gtsam_advanced_estimator.gtsam_operator(rm,object_vertex_array)
    my_estimator_no_vision = test_gtsam_advanced_estimator.gtsam_operator(rm,object_vertex_array)
    my_estimator_no_vision.use_live_vision = False

    msg_list = []
    time_list = []

    object_vertex_array_apriltag = None
    object_vertex_array_fast_vision = None
    object_vertex_array_estimated_with_vision = None
    object_vertex_array_estimated_no_vision = None

    num_vertices = None
    num_error_points = 0

    E_fast_vision = 0.0
    E_est_with_vision = 0.0
    E_est_no_vision = 0.0

    while rm.read_still_running():
        rm.unpack_all()

        my_estimator_with_vision.update_estimator()
        my_estimator_no_vision.update_estimator()

        if my_estimator_with_vision.can_publish:
            object_vertex_array_estimated_with_vision = my_estimator_with_vision.polygon_contact_dict['vertex_array_out'][0:2,:]

        if my_estimator_no_vision.can_publish:
            object_vertex_array_estimated_no_vision = my_estimator_no_vision.polygon_contact_dict['vertex_array_out'][0:2,:]

        if rm.polygon_vision_estimate_dict is not None:
            object_vertex_array_fast_vision = rm.polygon_vision_estimate_dict['vertex_array'][0:2,:]
     
        apriltag_id = ctm.find_valid_apriltag_id(rm.apriltag_pose_list_dict)
        object_vertex_array = ctm.get_object_vertex_array()

        if apriltag_id is not None:
            obj_pose_homog = ctm.generate_obj_pose_from_apriltag(rm.apriltag_pose_list_dict)
            object_vertex_array_apriltag = np.dot(obj_pose_homog,object_vertex_array)[0:2,:]


        num_vertices = len(object_vertex_array[0])

        if (object_vertex_array_apriltag is not None and object_vertex_array_fast_vision is not None
            and object_vertex_array_estimated_with_vision is not None and object_vertex_array_estimated_no_vision is not None):

            delta_E_fast_vision = compute_error(object_vertex_array_apriltag,object_vertex_array_fast_vision)
            delta_E_est_with_vision = compute_error(object_vertex_array_apriltag,object_vertex_array_estimated_with_vision)
            delta_E_est_no_vision = compute_error(object_vertex_array_apriltag,object_vertex_array_estimated_no_vision)

            if (delta_E_fast_vision is not None and
                delta_E_est_with_vision is not None and
                delta_E_est_no_vision is not None and
                rm.measured_contact_wrench[0]>2.0):
                
                E_fast_vision += delta_E_fast_vision
                E_est_with_vision += delta_E_est_with_vision
                E_est_no_vision += delta_E_est_no_vision
                num_error_points+=1


        rm.sleep()

    MSE_fast_vision = E_fast_vision/(num_vertices*num_error_points)
    MSE_est_with_vision = E_est_with_vision/(num_vertices*num_error_points)
    MSE_est_no_vision = E_est_no_vision/(num_vertices*num_error_points)

    print(np.sqrt(MSE_fast_vision))
    print(np.sqrt(MSE_est_with_vision))
    print(np.sqrt(MSE_est_no_vision))



def wall_pivot_ablation(fname_in,read_path, frame_rate, ground_x, wall_y):
    rm = ros_manager(load_mode = True, path=read_path, fname=fname_in)

    rm.setRate(frame_rate)
    rm.spawn_transform_listener()
    rm.subscribe_to_list(subscribe_required_list,True)
    rm.subscribe_to_list(subscribe_not_required_list,False)


    shape_name_list = ['big_triangle','big_square','rectangle','square','triangle','big_rectangle','rectangle_bump_in','rectangle_bump_out']

    cam_choice = 'near'

    ctm = camera_transform_manager(rm,cam_choice)
    ctm.setup_frames()
    camera_transformation_matrix = ctm.generate_camera_transformation_matrix()
    shape_dict = ctm.load_shape_data(shape_name_list)

    rm.wait_for_necessary_data()


    object_vertex_array = test_gtsam_advanced_estimator.get_shape_prior(load_mode = True, path = read_path, fname = fname_in)
    print(object_vertex_array)


    my_fast_vision_generator = fast_vision_polygon_estimator_node.polygon_vision_operator(rm)
    my_estimator_with_vision = test_gtsam_advanced_estimator.gtsam_operator(rm,object_vertex_array)
    my_estimator_no_vision = test_gtsam_advanced_estimator.gtsam_operator(rm,object_vertex_array)
    my_estimator_no_vision.use_live_vision = False

    my_estimator_with_vision.allow_corner_contact = False
    my_estimator_no_vision.allow_corner_contact = False

    msg_list = []
    time_list = []

    object_vertex_array_apriltag = None
    object_vertex_array_fast_vision = None
    object_vertex_array_estimated_with_vision = None
    object_vertex_array_estimated_no_vision = None

    num_vertices = None
    num_error_points = 0

    E_fast_vision = 0.0
    E_est_with_vision = 0.0
    E_est_no_vision = 0.0

    wall_contact_force_margin = 3.0

   

    while rm.read_still_running():
        rm.unpack_all()

        if len(rm.friction_parameter_dict['aer'])>0:
            wall_contact_right_bool = any(np.dot(rm.friction_parameter_dict['aer'],rm.measured_world_manipulation_wrench) > rm.friction_parameter_dict['ber'] + wall_contact_force_margin)
        else:
            wall_contact_right_bool = True

        if len(rm.friction_parameter_dict['ael'])>0:
            wall_contact_left_bool = any(np.dot(rm.friction_parameter_dict['ael'],rm.measured_world_manipulation_wrench) > rm.friction_parameter_dict['bel'] + wall_contact_force_margin)
        else:
            wall_contact_left_bool = True

        if wall_contact_right_bool and not wall_contact_left_bool:
            wall_flag = 0
        elif not wall_contact_right_bool and wall_contact_left_bool:
            wall_flag = 1
        elif wall_contact_right_bool and wall_contact_left_bool:
            wall_flag = 2

        wall_contact_on = wall_contact_right_bool or wall_contact_left_bool

 

        my_estimator_with_vision.update_estimator()
        my_estimator_no_vision.update_estimator()



        if my_estimator_with_vision.can_publish:
            object_vertex_array_estimated_with_vision = my_estimator_with_vision.polygon_contact_dict['vertex_array_out'][0:2,:]

        if my_estimator_no_vision.can_publish:
            object_vertex_array_estimated_no_vision = my_estimator_no_vision.polygon_contact_dict['vertex_array_out'][0:2,:]

        if rm.polygon_vision_estimate_dict is not None:
            object_vertex_array_fast_vision = rm.polygon_vision_estimate_dict['vertex_array'][0:2,:]
     
        


        num_vertices = len(object_vertex_array_estimated_no_vision[0])

        if (object_vertex_array_fast_vision is not None and object_vertex_array_estimated_with_vision is not None and object_vertex_array_estimated_no_vision is not None
            and wall_contact_on):

            x_estimate_vision = min(object_vertex_array_estimated_with_vision[0])
            x_estimate_no_vision = min(object_vertex_array_estimated_no_vision[0])
            x_fast_vision = min(object_vertex_array_fast_vision[0])

            y_estimate_vision = min(object_vertex_array_estimated_with_vision[1])
            y_estimate_no_vision = min(object_vertex_array_estimated_no_vision[1])
            y_fast_vision = min(object_vertex_array_fast_vision[1])

            print(x_estimate_vision, x_estimate_no_vision, x_fast_vision)
            print(y_estimate_vision, y_estimate_no_vision, y_fast_vision)

            E_est_with_vision += (x_estimate_vision-ground_x)**2 + (y_estimate_vision-wall_y)**2
            E_est_no_vision += (x_estimate_no_vision-ground_x)**2 + (y_estimate_no_vision-wall_y)**2
            E_fast_vision += (x_fast_vision-ground_x)**2 + (y_fast_vision-wall_y)**2

            num_error_points+=1


        rm.sleep()

    RMSE_fast_vision = np.sqrt(E_fast_vision/(num_error_points*2))
    RMSE_est_with_vision = np.sqrt(E_est_with_vision/(num_error_points*2))
    RMSE_est_no_vision = np.sqrt(E_est_no_vision/(num_error_points*2))

    print('RMSE_fast_vision: ',RMSE_fast_vision)
    print('RMSE_est_with_vision: ',RMSE_est_with_vision)
    print('RMSE_est_no_vision: ',RMSE_est_no_vision)



def hand_corner_rotate_ablation(fname_in,read_path, frame_rate, ground_x, wall_y,rect_w,rect_h):
    rm = ros_manager(load_mode = True, path=read_path, fname=fname_in)

    rm.setRate(frame_rate)
    rm.spawn_transform_listener()
    rm.subscribe_to_list(subscribe_required_list,True)
    rm.subscribe_to_list(subscribe_not_required_list,False)


    shape_name_list = ['big_triangle','big_square','rectangle','square','triangle','big_rectangle','rectangle_bump_in','rectangle_bump_out']

    cam_choice = 'near'

    ctm = camera_transform_manager(rm,cam_choice)
    ctm.setup_frames()
    camera_transformation_matrix = ctm.generate_camera_transformation_matrix()
    shape_dict = ctm.load_shape_data(shape_name_list)

    rm.wait_for_necessary_data()


    object_vertex_array = test_gtsam_advanced_estimator.get_shape_prior(load_mode = True, path = read_path, fname = fname_in)
    print(object_vertex_array)


    my_fast_vision_generator = fast_vision_polygon_estimator_node.polygon_vision_operator(rm)
    my_estimator_with_vision = test_gtsam_advanced_estimator.gtsam_operator(rm,object_vertex_array)
    my_estimator_no_vision = test_gtsam_advanced_estimator.gtsam_operator(rm,object_vertex_array)
    my_estimator_no_vision.use_live_vision = False

    # my_estimator_with_vision.allow_corner_contact = False
    # my_estimator_no_vision.allow_corner_contact = False

    msg_list = []
    time_list = []

    object_vertex_array_apriltag = None
    object_vertex_array_fast_vision = None
    object_vertex_array_estimated_with_vision = None
    object_vertex_array_estimated_no_vision = None

    num_vertices = None
    num_error_points = 0

    E_fast_vision = 0.0
    E_est_with_vision = 0.0
    E_est_no_vision = 0.0

    wall_contact_force_margin = 3.0

   

    while rm.read_still_running():
        rm.unpack_all()

        if len(rm.friction_parameter_dict['aer'])>0:
            wall_contact_right_bool = any(np.dot(rm.friction_parameter_dict['aer'],rm.measured_world_manipulation_wrench) > rm.friction_parameter_dict['ber'] + wall_contact_force_margin)
        else:
            wall_contact_right_bool = True

        if len(rm.friction_parameter_dict['ael'])>0:
            wall_contact_left_bool = any(np.dot(rm.friction_parameter_dict['ael'],rm.measured_world_manipulation_wrench) > rm.friction_parameter_dict['bel'] + wall_contact_force_margin)
        else:
            wall_contact_left_bool = True

        if wall_contact_right_bool and not wall_contact_left_bool:
            wall_flag = 0
        elif not wall_contact_right_bool and wall_contact_left_bool:
            wall_flag = 1
        elif wall_contact_right_bool and wall_contact_left_bool:
            wall_flag = 2

        wall_contact_on = wall_contact_right_bool or wall_contact_left_bool

 

        my_estimator_with_vision.update_estimator()
        my_estimator_no_vision.update_estimator()



        if my_estimator_with_vision.can_publish:
            object_vertex_array_estimated_with_vision = my_estimator_with_vision.polygon_contact_dict['vertex_array_out'][0:2,:]

        if my_estimator_no_vision.can_publish:
            object_vertex_array_estimated_no_vision = my_estimator_no_vision.polygon_contact_dict['vertex_array_out'][0:2,:]

        if rm.polygon_vision_estimate_dict is not None:
            object_vertex_array_fast_vision = rm.polygon_vision_estimate_dict['vertex_array'][0:2,:]
     
        


        num_vertices = len(object_vertex_array_estimated_no_vision[0])

        if (object_vertex_array_fast_vision is not None and object_vertex_array_estimated_with_vision is not None and object_vertex_array_estimated_no_vision is not None
            and wall_contact_on and
            len(object_vertex_array_fast_vision[0])==4 and len(object_vertex_array_estimated_with_vision[0])==4 and len(object_vertex_array_estimated_no_vision[0])==4):

            # ref_vertices = np.array([[ground_x, ground_x,       ground_x+rect_h,   ground_x+rect_h],
            #                          [wall_y,   wall_y+rect_w,  wall_y,            wall_y+rect_w]])

            ref_vertex = np.array([ground_x+rect_h,wall_y+rect_w])

            # x_estimate_vision = min(object_vertex_array_estimated_with_vision[0])
            # x_estimate_no_vision = min(object_vertex_array_estimated_no_vision[0])
            # x_fast_vision = min(object_vertex_array_fast_vision[0])

            # y_estimate_vision = min(object_vertex_array_estimated_with_vision[1])
            # y_estimate_no_vision = min(object_vertex_array_estimated_no_vision[1])
            # y_fast_vision = min(object_vertex_array_fast_vision[1])

            # print(x_estimate_vision, x_estimate_no_vision, x_fast_vision)
            # print(y_estimate_vision, y_estimate_no_vision, y_fast_vision)

            # E_est_with_vision += (x_estimate_vision-ground_x)**2 + (y_estimate_vision-wall_y)**2
            # E_est_no_vision += (x_estimate_no_vision-ground_x)**2 + (y_estimate_no_vision-wall_y)**2
            # E_fast_vision += (x_fast_vision-ground_x)**2 + (y_fast_vision-wall_y)**2


            delta_E_fast_vision = compute_error2(ref_vertex,object_vertex_array_fast_vision)
            delta_E_est_with_vision = compute_error2(ref_vertex,object_vertex_array_estimated_with_vision)
            delta_E_est_no_vision = compute_error2(ref_vertex,object_vertex_array_estimated_no_vision)



            if (delta_E_fast_vision is not None and
                delta_E_est_with_vision is not None and
                delta_E_est_no_vision is not None and
                rm.measured_contact_wrench[0]>2.0):
                
                E_fast_vision += delta_E_fast_vision
                E_est_with_vision += delta_E_est_with_vision
                E_est_no_vision += delta_E_est_no_vision

                num_error_points+=1


        rm.sleep()

    RMSE_fast_vision = np.sqrt(E_fast_vision/(num_error_points))
    RMSE_est_with_vision = np.sqrt(E_est_with_vision/(num_error_points))
    RMSE_est_no_vision = np.sqrt(E_est_no_vision/(num_error_points))

    print('RMSE_fast_vision: ',RMSE_fast_vision)
    print('RMSE_est_with_vision: ',RMSE_est_with_vision)
    print('RMSE_est_no_vision: ',RMSE_est_no_vision)


if __name__ == '__main__':
    fname_in = '/test_data-experiment0004.pickle'
    read_path = '/home/thecube/Documents/pbal_experiments/IROS_ablation_data'
    frame_rate = 10

    # update_pickle_with_fast_vision(fname_in,read_path, 10)



    # wall_pivot_ablation('/test_data-experiment0002.pickle',read_path, 10, ground_x = 0.0, wall_y = -.25144)
    # wall_pivot_ablation('/test_data-experiment0003.pickle',read_path, 10, ground_x = .04358, wall_y = -.22855)
    hand_corner_rotate_ablation('/test_data-experiment0004.pickle',read_path, 10, ground_x = 0.0, wall_y = -.1623, rect_w = .11,rect_h = .18)