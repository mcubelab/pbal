import numpy as np

def get_outward_normals(object_vertex_array):
    wrap_angle = 0.0

    num_vertices = len(object_vertex_array[0])
    for i in range(num_vertices):
        dx0 = object_vertex_array[0,(i+1)%num_vertices]-object_vertex_array[0,i]
        dy0 = object_vertex_array[1,(i+1)%num_vertices]-object_vertex_array[1,i]
        norm0 = np.sqrt(dx0**2+dy0**2)
        dx0/=norm0
        dy0/=norm0

        dx1 = object_vertex_array[0,(i+2)%num_vertices]-object_vertex_array[0,(i+1)%num_vertices]
        dy1 = object_vertex_array[1,(i+2)%num_vertices]-object_vertex_array[1,(i+1)%num_vertices]
        norm1 = np.sqrt(dx1**2+dy1**2)
        dx1/=norm1
        dy1/=norm1

        wrap_angle+= np.arcsin(dx0*dy1-dy0*dx1)

    object_normal_array = 0.0*object_vertex_array

    for i in range(num_vertices):
        dx0 = object_vertex_array[0,(i+1)%num_vertices]-object_vertex_array[0,i]
        dy0 = object_vertex_array[1,(i+1)%num_vertices]-object_vertex_array[1,i]
        norm0 = np.sqrt(dx0**2+dy0**2)
        dx0/=norm0
        dy0/=norm0

        if wrap_angle>0:
            object_normal_array[0,i]=dy0
            object_normal_array[1,i]=-dx0
        else:
            object_normal_array[0,i]=-dy0
            object_normal_array[1,i]=dx0

    return object_normal_array


def identify_contact_face_from_raw_data(object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog):
    
    hand_normal = np.array([[1.0], [0.0], [0.0], [0.0]])
    hand_tangent = np.array([[0.0], [1.0], [0.0], [0.0]])
    
    object_normal_array = get_outward_normals(object_vertex_array)

    hand_normal_world = np.dot(ee_pose_in_world_manipulation_homog,hand_normal)
    hand_tangent_world = np.dot(ee_pose_in_world_manipulation_homog,hand_tangent)

    object_vertex_array_world = np.dot(obj_pose_homog,object_vertex_array)
    object_normal_array_world = np.dot(obj_pose_homog,object_normal_array)
    contact_face = identify_contact_face(object_normal_array_world,hand_normal_world)

    return contact_face



def identify_contact_face(object_normal_array_world,hand_normal_world):
    min_val = np.inf
    current_face = None

    for i in range(len(object_normal_array_world[0])):
        compare_val = np.dot(object_normal_array_world[:3,i],hand_normal_world[:3,0])
        if compare_val<min_val:
            min_val = compare_val
            current_face = i

    return current_face

#reorients object_vertex_array and object_normal_array such that
#object_normal_array[:2,contact_face]=[-1,0]
#in other words, the outward facing surface normal of the object is pointing
#in the -x direction (i.e. it's pointing into the palm of the hand)
def reorient_vertices(object_vertex_array,object_normal_array,contact_face):
    nx = object_normal_array[0,contact_face]
    ny = object_normal_array[1,contact_face]

    rot_mat = np.array([[-nx,-ny,0.0,0.0],
                        [ ny,-nx,0.0,0.0],
                        [0.0,0.0,1.0,0.0],
                        [0.0,0.0,0.0,1.0]])

    object_vertex_array = np.dot(rot_mat,object_vertex_array)
    object_normal_array = np.dot(rot_mat,object_normal_array)

    return object_vertex_array,object_normal_array

def generate_shape_prior(object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog):
    
    hand_normal = np.array([[1.0], [0.0], [0.0], [0.0]])
    hand_tangent = np.array([[0.0], [1.0], [0.0], [0.0]])
    
    object_normal_array = get_outward_normals(object_vertex_array)

    hand_normal_world = np.dot(ee_pose_in_world_manipulation_homog,hand_normal)
    hand_tangent_world = np.dot(ee_pose_in_world_manipulation_homog,hand_tangent)

    object_vertex_array_world = np.dot(obj_pose_homog,object_vertex_array)
    object_normal_array_world = np.dot(obj_pose_homog,object_normal_array)
    contact_face = identify_contact_face(object_normal_array_world,hand_normal_world)

    test_object_vertex_array,test_object_normal_array = reorient_vertices(object_vertex_array,object_normal_array,contact_face)

    contact_vertex_world = object_vertex_array_world[:,contact_face]
    # print('contact_vertex_world = ',contact_vertex_world)

    hand_pos_world = ee_pose_in_world_manipulation_homog[:,3]
    # print('hand_pos_world = ', hand_pos_world)

    pos_diff_vertex_hand = contact_vertex_world-hand_pos_world
    # print('pos_diff_vertex_hand', pos_diff_vertex_hand)

    # print('hand_tangent_world = ', hand_tangent_world)

    dt = np.dot(pos_diff_vertex_hand,hand_tangent_world.transpose()[0])
    # print('dt = ',dt)

    test_object_vertex_array[0]-=test_object_vertex_array[0][contact_face]
    test_object_vertex_array[1]-=test_object_vertex_array[1][contact_face]
    test_object_vertex_array[1]+=dt

    return test_object_vertex_array, test_object_normal_array

def generate_shape_prior_for_advanced_estimator(object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog):
    num_vertices = len(object_vertex_array[0])

    dict_out = {}

    contact_face = identify_contact_face_from_raw_data(object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog)
    test_object_vertex_array, test_object_normal_array = generate_shape_prior(object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog)

    mean_val = np.array([0.0,0.0])

    for i in range(2):
        mean_val[i] = np.mean(test_object_vertex_array[i])
        test_object_vertex_array[i]-=mean_val[i]

    d_offset_list = np.array([0.0]*num_vertices)
    s_offset_list = np.array([0.0]*num_vertices)
    s_offset_list[contact_face] = mean_val[1]
    theta_offset_list = np.array([0.0]*num_vertices)


    for i in range(num_vertices):    
        d_offset_list[i] = np.dot(test_object_vertex_array[:,i],test_object_normal_array[:,i])
        theta_offset_list[i] = np.arctan2(test_object_normal_array[1,i],-test_object_normal_array[0,i])

    dict_out['contact_face'] = contact_face
    dict_out['d_offset_list'] = d_offset_list
    dict_out['s_offset_list'] = s_offset_list
    dict_out['theta_offset_list'] = theta_offset_list
    dict_out['test_object_vertex_array'] = test_object_vertex_array
    dict_out['test_object_normal_array'] = test_object_normal_array
    dict_out['num_vertices'] = num_vertices

    # answer check just to make sure the math is consistent
    # for i in range(num_vertices):
    #     theta_obj_in_ee = theta_offset_list[i]
    #     rot_mat_obj = np.array([[ np.cos(theta_obj_in_ee), -np.sin(theta_obj_in_ee)], 
    #                             [ np.sin(theta_obj_in_ee),  np.cos(theta_obj_in_ee)]])
    #     answer_check_normal = np.dot(rot_mat_obj,test_object_normal_array[0:2,i])
    #     print(answer_check_normal)

    #     d_test = np.dot(rot_mat_obj,test_object_vertex_array[0:2,i])[0]
    #     print(d_test+d_offset_list[i])

    return dict_out

def generate_shape_prior_for_advanced_estimator_floating(test_object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog):
    num_vertices = len(test_object_vertex_array[0])

    dict_out = {}

    # contact_face = identify_contact_face_from_raw_data(object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog)
    # test_object_vertex_array, test_object_normal_array = generate_shape_prior(object_vertex_array,obj_pose_homog,ee_pose_in_world_manipulation_homog)

    test_object_normal_array = get_outward_normals(test_object_vertex_array)

    mean_val = np.array([0.0,0.0])

    # for i in range(2):
    #     mean_val[i] = np.mean(test_object_vertex_array[i])
    #     test_object_vertex_array[i]-=mean_val[i]

    d_offset_list = np.array([0.0]*num_vertices)
    # s_offset_list = np.array([0.0]*num_vertices)
    # s_offset_list[contact_face] = mean_val[1]
    theta_offset_list = np.array([0.0]*num_vertices)


    for i in range(num_vertices):    
        d_offset_list[i] = np.dot(test_object_vertex_array[:,i],test_object_normal_array[:,i])
        theta_offset_list[i] = np.arctan2(test_object_normal_array[1,i],-test_object_normal_array[0,i])

    # dict_out['contact_face'] = contact_face
    dict_out['d_offset_list'] = d_offset_list
    # dict_out['s_offset_list'] = s_offset_list
    dict_out['theta_offset_list'] = theta_offset_list
    dict_out['test_object_vertex_array'] = test_object_vertex_array
    dict_out['test_object_normal_array'] = test_object_normal_array
    dict_out['num_vertices'] = num_vertices

    return dict_out

def determine_contact_vertices_floating(theta_obj,test_object_vertex_array,measured_world_manipulation_wrench,threshold_val = .015):

    rot_mat = np.array([[np.cos(theta_obj),-np.sin(theta_obj),0.0,0.0],
                        [np.sin(theta_obj), np.cos(theta_obj),0.0,0.0],
                        [              0.0,               0.0,1.0,0.0],
                        [              0.0,               0.0,0.0,1.0]])

    threshold_mat = np.dot(rot_mat,test_object_vertex_array)
    height_indices = np.argsort(threshold_mat[0])

    if np.abs(threshold_mat[0,height_indices[0]]-threshold_mat[0,height_indices[1]])>threshold_val:
        return height_indices[:1]
    else:
        return height_indices[:2]  


def determine_contact_vertices(theta_hand,test_object_vertex_array,measured_world_manipulation_wrench,threshold_val = .015):

    rot_mat = np.array([[-np.cos(theta_hand), np.sin(theta_hand),0.0,0.0],
                        [-np.sin(theta_hand),-np.cos(theta_hand),0.0,0.0],
                        [                0.0,                0.0,1.0,0.0],
                        [                0.0,                0.0,0.0,1.0]])

    threshold_mat = np.dot(rot_mat,test_object_vertex_array)
    height_indices = np.argsort(threshold_mat[0])

    if np.abs(threshold_mat[0,height_indices[0]]-threshold_mat[0,height_indices[1]])>threshold_val:
        return height_indices[:1]
    else:
        return height_indices[:2]  

def determine_wall_contact_vertices_for_controller(vertex_array_wm,measured_world_manipulation_wrench):
    vertex_array_wm = vertex_array_wm[0:3,:]

    direction_vec = -np.array([measured_world_manipulation_wrench[0],measured_world_manipulation_wrench[1],0.0])

    if np.linalg.norm(direction_vec)<2.0:
        direction_vec = np.array([1.0,0.0,0.0])
    else:
        direction_vec/=np.linalg.norm(direction_vec)

    threshold_mat = np.dot(direction_vec,vertex_array_wm)
    contact_indices = np.argsort(threshold_mat)

    return contact_indices[:2]


def estimate_external_COP(theta_hand, s_hand, measured_world_manipulation_wrench, test_object_vertex_array, contact_indices):

    # print('s_hand: ', s_hand)
    # s_hand = 0.0
    if s_hand is None:
        s_hand = 0.0

    rot_mat = np.array([[-np.cos(theta_hand), np.sin(theta_hand),0.0,0.0],
                        [-np.sin(theta_hand),-np.cos(theta_hand),0.0,0.0],
                        [                0.0,                0.0,1.0,0.0],
                        [                0.0,                0.0,0.0,1.0]])

    temp_vertices = np.array(test_object_vertex_array)
    temp_vertices[1]-=s_hand

    obj_vertices_world_manipulation = np.dot(rot_mat,temp_vertices)

    P_a = obj_vertices_world_manipulation[:, contact_indices[0]]
    P_b = obj_vertices_world_manipulation[:, contact_indices[1]]
    P_e = np.array([0.0,0.0,0.0])

    moment_arm_a = P_e[0:2] - P_a[0:2]
    moment_arm_b = P_e[0:2] - P_b[0:2]

    fn_wm  = measured_world_manipulation_wrench[0]
    ft_wm  = measured_world_manipulation_wrench[1]
    tau_wm = measured_world_manipulation_wrench[2]


    Tau_a = moment_arm_a[0]*ft_wm-moment_arm_a[1]*fn_wm+tau_wm
    Tau_b = moment_arm_b[0]*ft_wm-moment_arm_b[1]*fn_wm+tau_wm

    # print(moment_arm_a,moment_arm_b,measured_world_manipulation_wrench)
    # print(Tau_a,Tau_b)
    # print(moment_arm_a[0]*ft_wm,moment_arm_b[0]*ft_wm)
    # print(-moment_arm_a[1]*fn_wm,-moment_arm_b[1]*fn_wm)

    alpha0 = Tau_b/(Tau_b-Tau_a)
    alpha1 = Tau_a/(Tau_a-Tau_b)


    # P0 = alpha1 * P_a + alpha2 * P_b
    return alpha0, alpha1


