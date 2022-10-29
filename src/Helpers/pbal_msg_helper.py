import numpy as np
from pbal.msg import   (SlidingStateStamped, 
                        FrictionParamsStamped, 
                        ControlCommandStamped, 
                        QPDebugStamped,
                        PivotSlidingCommandedFlagStamped, 
                        TorqueConeBoundaryFlagStamped, 
                        TorqueConeBoundaryTestStamped,
                        TorqueBoundsStamped, 
                        GeneralizedPositionsStamped)

from geometry_msgs.msg import   (PoseStamped, 
                                TransformStamped, 
                                Pose2D, 
                                WrenchStamped,
                                PointStamped)

def list2pose_stamped(pose, frame_id="world"):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.position.z = pose[2]
    msg.pose.orientation.x = pose[3]
    msg.pose.orientation.y = pose[4]
    msg.pose.orientation.z = pose[5]
    msg.pose.orientation.w = pose[6]
    return msg

def pose_stamped2list(msg):
    return [float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w)]

def list2transform_stamped(transform, header_frame_id = 'base', child_frame_id = 'hand_estimate'):
    msg = TransformStamped()
    msg.header.frame_id = header_frame_id
    msg.child_frame_id = child_frame_id
    msg.transform.translation.x = transform[0]
    msg.transform.translation.y = transform[1]
    msg.transform.translation.z = transform[2]
    msg.transform.rotation.x = transform[3]
    msg.transform.rotation.y = transform[4]
    msg.transform.rotation.z = transform[5]
    msg.transform.rotation.w = transform[6]
    return msg

def transform_stamped2list(msg):
    return [float(msg.transform.translation.x),
            float(msg.transform.translation.y),
            float(msg.transform.translation.z),
            float(msg.transform.rotation.x),
            float(msg.transform.rotation.y),
            float(msg.transform.rotation.z),
            float(msg.transform.rotation.w)]

def list2wrench_stamped(wrench, frame_id='base'):
    msg = WrenchStamped()
    msg.header.frame_id = frame_id
    msg.wrench.force.x = wrench[0]
    msg.wrench.force.y = wrench[1]
    msg.wrench.force.z = wrench[2]
    msg.wrench.torque.x = wrench[3]
    msg.wrench.torque.y = wrench[4]
    msg.wrench.torque.z = wrench[5]
    return msg

def wrench_stamped2list(msg):
    return [float(msg.wrench.force.x), 
            float(msg.wrench.force.y), 
            float(msg.wrench.force.z), 
            float(msg.wrench.torque.x), 
            float(msg.wrench.torque.y), 
            float(msg.wrench.torque.z)]

def wrenchstamped_2FT(msg):
    force = [float(msg.wrench.force.x),
             float(msg.wrench.force.y),
             float(msg.wrench.force.z)]

    torque = [float(msg.wrench.torque.x),
              float(msg.wrench.torque.y),
              float(msg.wrench.torque.z)]

    return force, torque

def list2point_stamped(xyz):
    msg = PointStamped()
    msg.point.x = xyz[0]
    msg.point.y = xyz[1]
    msg.point.z = xyz[2]
    return msg

def point_stamped2list(msg):
    return [float(msg.point.x), float(msg.point.y), float(msg.point.z)]

def list2pose_twod(pose):
    msg = Pose2D()
    msg.x = pose[0]
    msg.y = pose[1]
    msg.theta = pose[2]
    return msg

def pose_twod2list(msg):
    return [float(msg.x), float(msg.y), float(msg.theta)]

def quat2list(quat):
    return [float(quat.x), float(quat.y), float(quat.z), float(quat.w)]

def unit_pose_stamped():
    return list2pose_stamped([0.0,0.0,0.0,0.0,0.0,0.0,1.0])

def generate_torque_cone_boundary_flag_stamped(torque_boundary_flag):
    torque_boundary_flag_message = TorqueConeBoundaryFlagStamped()
    torque_boundary_flag_message.boundary_flag = torque_boundary_flag
    return torque_boundary_flag_message

def parse_torque_cone_boundary_flag_stamped(torque_boundary_flag_message):
    return torque_boundary_flag_message.boundary_flag

def generate_torque_cone_boundary_test_stamped(torque_boundary_boolean):
    torque_boundary_boolean_message = TorqueConeBoundaryTestStamped()
    torque_boundary_boolean_message.boundary_test = torque_boundary_boolean
    return torque_boundary_boolean_message

def parse_torque_cone_boundary_test_stamped(torque_boundary_boolean_message):
    return torque_boundary_boolean_message.boundary_test

def generate_pivot_sliding_commanded_flag(pivot_sliding_commanded_flag):
    pivot_sliding_commanded_flag_message = PivotSlidingCommandedFlagStamped()
    pivot_sliding_commanded_flag_message.command_flag = pivot_sliding_commanded_flag
    return pivot_sliding_commanded_flag_message

def parse_pivot_sliding_commanded_flag(pivot_sliding_commanded_flag_message):
    return pivot_sliding_commanded_flag_message.command_flag

def generate_torque_bounds_stamped(torque_bounds):
    torque_bounds_message = TorqueBoundsStamped()
    torque_bounds_message.torque_bounds = torque_bounds
    return torque_bounds_message

def parse_torque_bounds_stamped(torque_bounds_message):
    return torque_bounds_message.torque_bounds

def generate_generalized_positions_stamped(generalized_positions):
    generalized_positions_message = GeneralizedPositionsStamped()
    generalized_positions_message.generalized_positions = generalized_positions
    return generalized_positions_message

def parse_generalized_positions_stamped(generalized_positions_message):
    return generalized_positions_message.generalized_positions


def sliding_dict_to_sliding_stamped(sliding_dict):

    sliding_msg = SlidingStateStamped()

    sliding_msg.sliding_state.pivot_sliding_left = sliding_dict["pslf"]     # True if pivot sliding left
    sliding_msg.sliding_state.pivot_sliding_right  = sliding_dict["psrf"]   # True if pivot sliding right           
    sliding_msg.sliding_state.pivot_sliding = sliding_dict["psf"]           # pivot_sliding_left || pivot_sliding_right

    sliding_msg.sliding_state.robot_sliding_left = sliding_dict["cslf"]     # True if pivot sliding left
    sliding_msg.sliding_state.robot_sliding_right = sliding_dict["csrf"]    # True if pivot sliding right           
    sliding_msg.sliding_state.robot_sliding = sliding_dict["csf"]        # robot_sliding_left || robot_sliding_right

    return sliding_msg

def sliding_stamped_to_sliding_dict(sliding_msg):

    sliding_dict = {}

    sliding_dict["pslf"] = sliding_msg.sliding_state.pivot_sliding_left
    sliding_dict["psrf"] = sliding_msg.sliding_state.pivot_sliding_right
    sliding_dict["psf"] = sliding_msg.sliding_state.pivot_sliding

    sliding_dict["cslf"] = sliding_msg.sliding_state.robot_sliding_left
    sliding_dict["csrf"] = sliding_msg.sliding_state.robot_sliding_right
    sliding_dict["csf"] = sliding_msg.sliding_state.robot_sliding

    return sliding_dict

def friction_dict_to_friction_stamped(friction_dict):

    friction_msg = FrictionParamsStamped()

    # ground contact LHS
    friction_msg.friction_params.A_pivot_left = [
        item for row in friction_dict['ael'] for item in row]
    friction_msg.friction_params.b_pivot_left = friction_dict['bel']

    # ground contact RHS
    friction_msg.friction_params.A_pivot_right = [
        item for row in friction_dict['aer'] for item in row]
    friction_msg.friction_params.b_pivot_right = friction_dict['ber']

    # ground contact bools
    friction_msg.friction_params.use_env_left = friction_dict['elu']
    friction_msg.friction_params.use_env_right = friction_dict['eru']

    # hand contact LHS
    friction_msg.friction_params.A_robot_left = friction_dict['acl']
    friction_msg.friction_params.b_robot_left = friction_dict['bcl']

    # hand contact RHS
    friction_msg.friction_params.A_robot_right = friction_dict['acr'] 
    friction_msg.friction_params.b_robot_right = friction_dict['bcr']

    # hand contact bool
    friction_msg.friction_params.use_robot = friction_dict['cu']

    return friction_msg

def friction_stamped_to_friction_dict(friction_msg):

    friction_dict = {}

    # ground contact LHS
    friction_dict['ael'] = np.reshape(np.array(
        friction_msg.friction_params.A_pivot_left), newshape=(-1, 3)).tolist()
    friction_dict['bel'] = list(friction_msg.friction_params.b_pivot_left)

    # ground contact RHS

    friction_dict['aer'] = np.reshape(np.array(
        friction_msg.friction_params.A_pivot_right), newshape=(-1, 3)).tolist()
    friction_dict['ber'] = list(friction_msg.friction_params.b_pivot_right)

    # ground contact bools
    friction_dict['elu'] = friction_msg.friction_params.use_env_left
    friction_dict['eru'] = friction_msg.friction_params.use_env_right

    # hand contact LHS
    friction_dict['acl'] = list(friction_msg.friction_params.A_robot_left)
    friction_dict['bcl'] = friction_msg.friction_params.b_robot_left

    # hand contact RHS
    friction_dict['acr'] = list(friction_msg.friction_params.A_robot_right)
    friction_dict['bcr'] = friction_msg.friction_params.b_robot_right

    # hand contact bool
    friction_dict['cu'] = friction_msg.friction_params.use_robot

    return friction_dict

def command_stamped_to_command_dict(command_msg):
    
    command_dict = {}

    command_dict['name']            = command_msg.control_command.name
    command_dict['command_flag']    = command_msg.control_command.command_flag
    command_dict['mode']            = command_msg.control_command.mode

    # relative commands
    command_dict['delta_theta']     = command_msg.control_command.delta_theta
    command_dict['delta_s_pivot']   = command_msg.control_command.delta_s_pivot
    command_dict['delta_s_hand']    = command_msg.control_command.delta_s_hand

    # aboluste commands
    command_dict['theta']           = command_msg.control_command.theta
    command_dict['s_pivot']         = command_msg.control_command.s_pivot
    command_dict['s_hand']          = command_msg.control_command.s_hand


    return command_dict

def command_dict_to_command_stamped(command_dict):

    command_msg = ControlCommandStamped()

    if command_dict is None:
        return command_msg

    command_msg.control_command.name            = command_dict['name']
    command_msg.control_command.command_flag    = command_dict['command_flag']
    command_msg.control_command.mode            = command_dict['mode']


    if   'delta_theta' in command_dict:
        command_msg.control_command.delta_theta = command_dict['delta_theta']
    elif 'theta' in command_dict:
        command_msg.control_command.theta = command_dict['theta']
    else:
        raise RuntimeError("theta unspecified")

    if   'delta_s_pivot' in command_dict:
        command_msg.control_command.delta_s_pivot = command_dict['delta_s_pivot']
    elif 's_pivot' in command_dict:
        command_msg.control_command.s_pivot = command_dict['s_pivot']
    else:
        raise RuntimeError("s pivot unspecified")

    if   'delta_s_hand' in command_dict:
        command_msg.control_command.delta_s_hand = command_dict['delta_s_hand']
    elif 's_hand' in command_dict:
        command_msg.control_command.s_hand = command_dict['s_hand']
    else:
        raise RuntimeError("s hand unspecified")

    return command_msg

def qp_debug_dict_to_qp_debug_stamped(qp_debug_dict):

    qp_debug_msg = QPDebugStamped()

    # primitive info
    qp_debug_msg.qp_debug.name = qp_debug_dict['name']                              
    qp_debug_msg.qp_debug.mode = qp_debug_dict['mode']

    # break up error dict by keys/vals
    error_dict_keys = qp_debug_dict['error_dict'].keys()
    qp_debug_msg.qp_debug.error_dict_keys = ','.join(error_dict_keys)
    qp_debug_msg.qp_debug.error_dict_vals = [
        qp_debug_dict['error_dict'][key] for key in error_dict_keys]
    
    qp_debug_msg.qp_debug.snewrb  = qp_debug_dict['snewrb']                  

    # QP costs
    qp_debug_msg.qp_debug.quadratic_cost_term = [
        item for row in qp_debug_dict['quadratic_cost_term'] for item in row]          
    qp_debug_msg.qp_debug.linear_cost_term = qp_debug_dict['linear_cost_term']
    qp_debug_msg.qp_debug.proj_vec_list = [
        item for row in qp_debug_dict['proj_vec_list'] for item in row]
    qp_debug_msg.qp_debug.error_list = qp_debug_dict['error_list']

    # QP constraints
    qp_debug_msg.qp_debug.label_list_cnstr = ','.join(qp_debug_dict['label_list_cnstr'])
    qp_debug_msg.qp_debug.constraint_normals = [
        item for row in qp_debug_dict['constraint_normals'] for item in row]
    qp_debug_msg.qp_debug.constraint_offsets = qp_debug_dict['constraint_offsets']
    qp_debug_msg.qp_debug.measured_wrench = qp_debug_dict['measured_wrench']
    qp_debug_msg.qp_debug.slacks = qp_debug_dict['slacks']

    # Solution
    qp_debug_msg.qp_debug.solve_time = qp_debug_dict['solve_time']
    qp_debug_msg.qp_debug.delta_wrench = qp_debug_dict['delta_wrench']        
    qp_debug_msg.qp_debug.delta_wrench_unconstrained = qp_debug_dict[
        'delta_wrench_unconstrained']

    return qp_debug_msg           

def qp_debug_stamped_to_qp_debug_dict(qp_debug_msg):
    
    qp_dict = {}

    # primitive info
    qp_dict['name'] = qp_debug_msg.qp_debug.name
    qp_dict['mode'] = qp_debug_msg.qp_debug.mode

    # join dict back together
    error_dict_keys = qp_debug_msg.qp_debug.error_dict_keys.split(',')
    error_dict_vals = qp_debug_msg.qp_debug.error_dict_vals

    qp_dict['error_dict'] = {key : val
        for key, val in zip(error_dict_keys, error_dict_vals)}
  

    # QP costs
    qp_dict['quadratic_cost_term'] = np.reshape(np.array(
         qp_debug_msg.qp_debug.quadratic_cost_term), newshape=(-1, 3)).tolist()    
    qp_dict['linear_cost_term'] = qp_debug_msg.qp_debug.linear_cost_term
    qp_dict['proj_vec_list'] = np.reshape(np.array(
         qp_debug_msg.qp_debug.proj_vec_list), newshape=(-1, 3)).tolist()
    qp_dict['error_list'] = qp_debug_msg.qp_debug.error_list


    # QP constraints
    qp_dict['label_list_cnstr'] = qp_debug_msg.qp_debug.label_list_cnstr.split(',')
    qp_dict['constraint_normals'] = np.reshape(np.array(
         qp_debug_msg.qp_debug.constraint_normals), newshape=(-1, 3)).tolist() 
    qp_dict['constraint_offsets'] = qp_debug_msg.qp_debug.constraint_offsets
    qp_dict['measured_wrench'] = qp_debug_msg.qp_debug.measured_wrench
    qp_dict['slacks'] = qp_debug_msg.qp_debug.slacks

    # Solution
    qp_dict['solve_time'] = qp_debug_msg.qp_debug.solve_time
    qp_dict['delta_wrench'] = qp_debug_msg.qp_debug.delta_wrench
    qp_dict['delta_wrench_unconstrained'] = qp_debug_msg.qp_debug.delta_wrench_unconstrained

    return qp_dict


def ground_truth_stamped_to_ground_truth_dict(ground_truth_msg):

    ground_truth_dict = {}

    # robot kinematics
    ground_truth_dict['tht_h'] = ground_truth_msg.ground_truth.hand_angle
    ground_truth_dict['hp'] = ground_truth_msg.ground_truth.hand_pose
    ground_truth_dict['sq'] = ground_truth_msg.ground_truth.hand_sliding

    # object kinematics
    ground_truth_dict['tht_o'] = ground_truth_msg.ground_truth.object_angle
    ground_truth_dict['op'] = ground_truth_msg.ground_truth.object_pose
    ground_truth_dict['sg'] = ground_truth_msg.ground_truth.object_sliding
    ground_truth_dict['pivs'] = np.reshape(np.array(
        ground_truth_msg.ground_truth.pivot_locations), 
        newshape=(-1, 2)).tolist() 

    # ground 
    ground_truth_dict['tht_g'] = ground_truth_msg.ground_truth.ground_angle

    # perception 
    ground_truth_dict['sn'] = ground_truth_msg.ground_truth.shape_name
    ground_truth_dict['atid'] = ground_truth_msg.ground_truth.apriltag_id
    ground_truth_dict['od'] = ground_truth_msg.ground_truth.object_dectected

    return ground_truth_dict


def ground_truth_dict_to_ground_truth_stamped(ground_truth_dict):

    ground_truth_msg = GroundTruthStamped()

    # robot kinematics
    ground_truth_msg.ground_truth.hand_angle = ground_truth_dict['tht_h']
    ground_truth_msg.ground_truth.hand_pose = ground_truth_dict['hp']
    ground_truth_msg.ground_truth.hand_sliding = ground_truth_dict['sq']

    # object kinematics
    ground_truth_msg.ground_truth.object_angle = ground_truth_dict['tht_o']
    ground_truth_msg.ground_truth.object_pose = ground_truth_dict['op']
    ground_truth_msg.ground_truth.object_sliding = ground_truth_dict['sg']
    ground_truth_msg.ground_truth.pivot_locations = [
        item for row in ground_truth_dict['pivs'] for item in row]

    # ground 
    ground_truth_msg.ground_truth.ground_angle = ground_truth_dict['tht_g']

    # perception 
    ground_truth_msg.ground_truth.shape_name = ground_truth_dict['sn']
    ground_truth_msg.ground_truth.apriltag_id = ground_truth_dict['atid']
    ground_truth_msg.ground_truth.object_dectected = ground_truth_dict['od']

    return ground_truth_msg

def parse_apriltag_detection_array(apriltag_message):

    if len(apriltag_message.detections)>0:
        detection_dict = {}
        for i in range(len(apriltag_message.detections)):

            msg = apriltag_message.detections[i]
            
            detection_dict[msg.id[0]] = {
            'id': msg.id[0],
            'size': msg.size[0],
            'position': [msg.pose.pose.pose.position.x, msg.pose.pose.pose.position.y, 
                msg.pose.pose.pose.position.z],
            'orientation': [msg.pose.pose.pose.orientation.x, msg.pose.pose.pose.orientation.y, 
                msg.pose.pose.pose.orientation.z, msg.pose.pose.pose.orientation.w]
            }
        return detection_dict
    else:
        return None

def parse_camera_info(camera_info_message):
    camera_matrix = np.reshape(camera_info_message.P, (3, 4))
    return camera_matrix.tolist()