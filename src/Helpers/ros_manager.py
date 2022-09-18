#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import json
import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped, TransformStamped
from std_msgs.msg import Bool, Int32, Float32MultiArray, Float32
from pbal.msg import SlidingStateStamped, FrictionParamsStamped, ControlCommandStamped, QPDebugStamped
import ros_helper as rh
import pbal_msg_helper as pmh
import time
import numpy as np
from Estimation import friction_reasoning
import tf2_ros
	# '/ee_pose_in_world_from_franka_publisher'

	# '/ft_sensor_in_base_frame'
	# '/ft_sensor_in_end_effector_frame'
	# '/end_effector_sensor_in_end_effector_frame'
	# '/end_effector_sensor_in_base_frame'
	# '/torque_cone_boundary_test'
	# '/torque_cone_boundary_flag'

	# '/friction_parameters'
	

	# '/pivot_frame_realsense'
	# '/pivot_frame_estimated'
	# '/generalized_positions'
	# '/end_effector_sensor_in_end_effector_frame'
	# '/barrier_func_control_command'
	
	# '/torque_bound_message'
	
	# '/pivot_sliding_commanded_flag'
	# '/qp_debug_message'
	# '/target_frame'

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

class ros_manager(object):
	def __init__(self):
		self.data_available_list = []
		self.available_mask_list = []
		self.unpack_list = []

	def wait_for_necessary_data(self):
		print("Waiting to hear from essential subscribers")
		can_proceed = False

		while not can_proceed:
			time.sleep(.1)

			can_proceed = True
			for i in range(len(self.data_available_list)):
				can_proceed = can_proceed and (self.data_available_list[i] or self.available_mask_list[i])

	def unpack_all(self):
		for my_func in self.unpack_list:
			my_func()

	def subscribe_to_list(self,list_in, isNecessary = True):
		for topic in list_in:
			self.subscribe(topic,isNecessary)

	def spawn_publisher_list(self,list_in):
		for topic in list_in:
			self.spawn_publisher(topic)

	def subscribe(self,topic, isNecessary = True):
		self.available_mask_list.append(not isNecessary)
		self.data_available_list.append(False)

		if   topic ==  '/netft/netft_data':
			self.unpack_list.append(self.force_unpack)
			self.ft_wrench_in_ft_sensor_list = []
			self.ft_wrench_in_ft_sensor_available_index = len(self.data_available_list)-1
			self.force_has_new = False

			self.ft_wrench_in_ft_sensor = None

			self.ft_wrench_in_ft_sensor_frame_sub = rospy.Subscriber(
				topic, 
				WrenchStamped, 
				self.force_callback, 
				queue_size=1)

		elif topic == '/ee_pose_in_world_from_franka_publisher':
			# subscribe to ee pose data
			self.unpack_list.append(self.ee_pose_unpack)
			self.panda_hand_in_base_pose_list = []
			self.panda_hand_in_base_pose_available_index = len(self.data_available_list)-1
			self.ee_pose_has_new = False

			self.panda_hand_in_base_pose = None 
			self.base_z_in_panda_hand = None

			self.panda_hand_in_base_pose_sub = rospy.Subscriber(
				topic, 
				PoseStamped, 
				self.ee_pose_callback, 
				queue_size=1)

		elif topic == '/end_effector_sensor_in_end_effector_frame':
			self.unpack_list.append(self.end_effector_wrench_unpack)
			self.measured_contact_wrench_list = []
			self.measured_contact_wrench_available_index = len(self.data_available_list)-1
			self.end_effector_wrench_has_new = False

			self.measured_contact_wrench = None

			self.end_effector_wrench_sub = rospy.Subscriber(
				topic, 
				WrenchStamped,  
				self.end_effector_wrench_callback)

		elif topic == '/end_effector_sensor_in_base_frame':
			self.unpack_list.append(self.end_effector_wrench_base_frame_unpack)
			self.measured_base_wrench_list = []
			self.measured_base_wrench_available_index = len(self.data_available_list)-1
			self.end_effector_wrench_base_frame_has_new = False

			self.measured_base_wrench = None

			self.end_effector_wrench_base_frame_sub = rospy.Subscriber(
				topic, 
				WrenchStamped,  
				self.end_effector_wrench_base_frame_callback)

		elif topic == '/friction_parameters':
			self.unpack_list.append(self.friction_parameter_unpack)
			self.friction_parameter_list = []
			self.friction_parameter_available_index = len(self.data_available_list)-1
			self.friction_parameter_has_new = False

			self.friction_parameter_dict, dummy1 ,dummy2 = friction_reasoning.initialize_friction_dictionaries()

			self.friction_parameter_sub = rospy.Subscriber(
				topic, 
				FrictionParamsStamped, 
				self.friction_parameter_callback)

		elif topic == '/pivot_frame_realsense':
			self.unpack_list.append(self.pivot_xyz_realsense_unpack)
			self.pivot_xyz_realsense_list = []
			self.pivot_xyz_realsense_available_index = len(self.data_available_list)-1
			self.pivot_xyz_realsense_has_new = False

			self.pivot_xyz = None
			self.pivot_xyz_realsense = None
			self.pivot_xyz_estimated = None
			self.pivot_message_realsense_time = None
			self.pivot_message_estimated_time = None

			self.pivot_xyz_realsense_sub = rospy.Subscriber(
				topic, 
				TransformStamped, 
				self.pivot_xyz_realsense_callback)

		elif topic == '/pivot_frame_estimated':
			self.unpack_list.append(self.pivot_xyz_estimated_unpack)
			self.pivot_xyz_estimated_list = []
			self.pivot_xyz_estimated_available_index = len(self.data_available_list)-1
			self.pivot_xyz_estimated_has_new = False

			self.pivot_xyz = None
			self.pivot_xyz_realsense = None
			self.pivot_xyz_estimated = None
			self.pivot_message_realsense_time = None
			self.pivot_message_estimated_time = None

			self.pivot_xyz_estimated_sub = rospy.Subscriber(
				topic, 
				TransformStamped, 
				self.pivot_xyz_estimated_callback)

		elif topic == '/generalized_positions':
			self.unpack_list.append(self.generalized_positions_unpack)
			self.generalized_positions_list = []
			self.generalized_positions_available_index = len(self.data_available_list)-1
			self.generalized_positions_has_new = False

			self.generalized_positions = None
			self.state_not_exists_bool = True

			self.generalized_positions_sub = rospy.Subscriber(
				topic, 
				Float32MultiArray,  
				self.generalized_positions_callback)

		elif topic == '/barrier_func_control_command':
			self.unpack_list.append(self.barrier_func_control_command_unpack)
			self.barrier_func_control_command_list = []
			self.barrier_func_control_command_available_index = len(self.data_available_list)-1
			self.barrier_func_control_command_has_new = False

			self.command_msg = None

			self.control_command_sub = rospy.Subscriber(
				topic, 
				ControlCommandStamped, 
				self.barrier_func_control_command_callback)

		elif topic == '/torque_bound_message':
			self.unpack_list.append(self.torque_bound_unpack)
			self.torque_bound_list = []
			self.torque_bound_available_index = len(self.data_available_list)-1
			self.torque_bound_has_new = False

			self.torque_bounds = None

			self.torque_bound_sub = rospy.Subscriber(
				topic, 
				Float32MultiArray,  
				self.torque_bound_callback)

	def spawn_publisher(self,topic):

		if topic == '/ee_pose_in_world_from_franka_publisher':
			self.ee_pose_in_world_from_franka_pub = rospy.Publisher(
				topic, 
				PoseStamped, 
				queue_size = 10)

		elif topic == '/ft_sensor_in_base_frame':

			#wrench at the force torque sensor rotated into the base frame
			#Wrench measured by the ATI, multiplied by a rotation matrix (from ATI frame to world)
			self.ft_sensor_in_base_frame_pub = rospy.Publisher(
				topic, 
				WrenchStamped, 
				queue_size = 10)

		elif topic == '/ft_sensor_in_end_effector_frame':

			#wrench at the force torque sensor rotated into the end effector frame -Neel 7/5/2022
			#wrench measured by the ATI, multiplied by a rotation matrix (from ATI frame to end-effector)
			self.ft_sensor_in_end_effector_frame_pub = rospy.Publisher(
				topic, 
				WrenchStamped, 
				queue_size = 10)

		elif topic == '/end_effector_sensor_in_end_effector_frame':
			#wrench at the end-effector in the end effector coordinates -Neel 7/5/2022
			#wrench measured by the ATI, but the torque has been transformed using a different reference point,
			#specifically, the origin of the end-effector frame (should be palm center), and using end-effector basis
			self.end_effector_sensor_in_end_effector_frame_pub = rospy.Publisher(
				topic, 
				WrenchStamped, 
				queue_size = 10)

		elif topic == '/end_effector_sensor_in_base_frame':
			#wrench at the end-effector in the base coordinates -Neel 7/5/2022
			#wrench measured by the ATI, but the torque has been transformed using a different reference point,
			#specifically, the origin of the end-effector frame (should be palm center), and using BASE FRAME basis
			self.end_effector_sensor_in_base_frame_pub = rospy.Publisher(
				topic, 
				WrenchStamped, 
				queue_size = 10)

		elif topic == '/torque_cone_boundary_test':
			self.torque_boundary_boolean_message = Bool()
			self.torque_cone_boundary_test_pub = rospy.Publisher(
				topic, 
				Bool , 
				queue_size = 10)

		elif topic == '/torque_cone_boundary_flag':
			self.torque_boundary_flag_message = Int32()
			self.torque_cone_boundary_flag_pub = rospy.Publisher(
				topic, 
				Int32 , 
				queue_size = 10)

		elif topic == '/friction_parameters':
			self.friction_parameter_pub = rospy.Publisher(
				topic, 
				FrictionParamsStamped,
				queue_size = 10)

		elif topic == '/sliding_state':
			self.sliding_state_pub = rospy.Publisher(
				topic, 
				SlidingStateStamped,
				queue_size = 10)

		elif topic == '/pivot_sliding_commanded_flag':
			# setting up publisher for sliding
			self.pivot_sliding_commanded_flag_msg = Bool()
			self.pivot_sliding_commanded_flag_pub = rospy.Publisher(
				topic, 
				Bool, 
				queue_size=10)

		elif topic == '/qp_debug_message':
			self.qp_debug_message_pub = rospy.Publisher(
				topic, 
				QPDebugStamped,
				queue_size=10)

		elif topic == '/target_frame':
			# intialize impedance target frame
			self.frame_message = initialize_frame()
			self.target_frame_pub = rospy.Publisher(topic, 
				TransformStamped, 
				queue_size=10) 
			# set up transform broadcaster
			self.target_frame_broadcaster = tf2_ros.TransformBroadcaster()



	def pub_ee_pose_in_world_from_franka(self,ee_pose_in_world_list):
		ee_pose_in_world_pose_stamped = rh.list2pose_stamped(ee_pose_in_world_list)
		self.ee_pose_in_world_from_franka_pub.publish(ee_pose_in_world_pose_stamped)

	def pub_ft_sensor_in_base_frame(self,msg):
		self.ft_sensor_in_base_frame_pub.publish(msg)

	def pub_ft_sensor_in_end_effector_frame(self,msg):
		self.ft_sensor_in_end_effector_frame_pub.publish(msg)

	def pub_end_effector_sensor_in_end_effector_frame(self,msg):
		self.end_effector_sensor_in_end_effector_frame_pub.publish(msg)

	def pub_end_effector_sensor_in_base_frame(self,msg):
		self.end_effector_sensor_in_base_frame_pub.publish(msg)

	def pub_torque_cone_boundary_test(self,torque_boundary_boolean):
		self.torque_boundary_boolean_message.data = torque_boundary_boolean
		self.torque_cone_boundary_test_pub.publish(self.torque_boundary_boolean_message)

	def pub_torque_cone_boundary_flag(self,torque_boundary_flag):
		self.torque_boundary_flag_message.data = torque_boundary_flag
		self.torque_cone_boundary_flag_pub.publish(self.torque_boundary_flag_message)

	def pub_friction_parameter(self,friction_parameter_dict):
		friction_parameter_msg = pmh.friction_dict_to_friction_stamped(friction_parameter_dict)
		self.friction_parameter_pub.publish(friction_parameter_msg)

	def pub_sliding_state(self,sliding_state_dict):
		self.sliding_state_pub.publish(
			pmh.sliding_dict_to_sliding_stamped(sliding_dict=sliding_state_dict))

	def pub_pivot_sliding_commanded_flag(self,pivot_sliding_commanded_flag):
		self.pivot_sliding_commanded_flag_msg.data = pivot_sliding_commanded_flag
		self.pivot_sliding_commanded_flag_pub.publish(self.pivot_sliding_commanded_flag_msg)

	def pub_target_frame(self,waypoint_pose_list):
		update_frame(rh.list2pose_stamped(waypoint_pose_list),self.frame_message)
		self.target_frame_pub.publish(self.frame_message)
		self.target_frame_broadcaster.sendTransform(self.frame_message)

	def pub_qp_debug_message(self,debug_dict):
		self.qp_debug_message_pub.publish(pmh.qp_debug_dict_to_qp_debug_stamped(
			qp_debug_dict = debug_dict))

	def force_callback(self,data):
		self.ft_wrench_in_ft_sensor_list.append(data)
		if len(self.ft_wrench_in_ft_sensor_list)>1:
			self.ft_wrench_in_ft_sensor_list.pop(0)
		self.data_available_list[self.ft_wrench_in_ft_sensor_available_index]=True

	def force_unpack(self):
		if len(self.ft_wrench_in_ft_sensor_list)>0:
			self.ft_wrench_in_ft_sensor = self.ft_wrench_in_ft_sensor_list.pop(0)

			self.force_has_new = True
		else:
			self.force_has_new = False	

	def ee_pose_callback(self,data):
		self.panda_hand_in_base_pose_list.append(data)
		if len(self.panda_hand_in_base_pose_list)>1:
			self.panda_hand_in_base_pose_list.pop(0)
		self.data_available_list[self.panda_hand_in_base_pose_available_index]=True
		
	def ee_pose_unpack(self):
		if len(self.panda_hand_in_base_pose_list)>0:

			self.panda_hand_in_base_pose = self.panda_hand_in_base_pose_list.pop(0)

			self.base_z_in_panda_hand = rh.matrix_from_pose(
				self.panda_hand_in_base_pose)[2, :3]

			self.ee_pose_has_new = True
		else:
			self.ee_pose_has_new = False

	def end_effector_wrench_callback(self,data):
		self.measured_contact_wrench_list.append(data)
		if len(self.measured_contact_wrench_list)>1:
			self.measured_contact_wrench_list.pop(0)
		self.data_available_list[self.measured_contact_wrench_available_index]=True

	def end_effector_wrench_unpack(self):
		if  len(self.measured_contact_wrench_list)>0:

			end_effector_wrench = self.measured_contact_wrench_list.pop(0)

			measured_contact_wrench_6D = rh.wrench_stamped2list(
				end_effector_wrench)

			self.measured_contact_wrench = -np.array([
				measured_contact_wrench_6D[0], 
				measured_contact_wrench_6D[1],
				measured_contact_wrench_6D[-1]])

			self.end_effector_wrench_has_new = True
		else:
			self.end_effector_wrench_has_new = False

	def end_effector_wrench_base_frame_callback(self,data):
		self.measured_base_wrench_list.append(data)
		if len(self.measured_base_wrench_list)>1:
			self.measured_base_wrench_list.pop(0)
		self.data_available_list[self.measured_base_wrench_available_index]=True

	def end_effector_wrench_base_frame_unpack(self):
		if len(self.measured_base_wrench_list)>0:
			base_wrench = self.measured_base_wrench_list.pop(0)

			measured_base_wrench_6D = rh.wrench_stamped2list(
				base_wrench)

			self.measured_base_wrench = -np.array([
				measured_base_wrench_6D[0], 
				measured_base_wrench_6D[2],
				measured_base_wrench_6D[-2]])

			self.end_effector_wrench_base_frame_has_new = True
		else:
			self.end_effector_wrench_base_frame_has_new = False

	def friction_parameter_callback(self,data):
		self.friction_parameter_list.append(data)
		if len(self.friction_parameter_list)>1:
			self.friction_parameter_list.pop(0)
		self.data_available_list[self.friction_parameter_available_index]=True

	def friction_parameter_unpack(self):
		if len(self.friction_parameter_list)>0:
			data = self.friction_parameter_list.pop(0)
			self.friction_parameter_dict = pmh.friction_stamped_to_friction_dict(data)
			friction_reasoning.convert_friction_param_dict_to_array(self.friction_parameter_dict)

			self.friction_parameter_has_new = True
		else:
			self.friction_parameter_has_new = False

	def pivot_xyz_realsense_callback(self,data):
		self.pivot_xyz_realsense_list.append([data,time.time()])
		if len(self.pivot_xyz_realsense_list)>1:
			self.pivot_xyz_realsense_list.pop(0)
		self.data_available_list[self.pivot_xyz_realsense_available_index]=True

	def pivot_xyz_realsense_unpack(self):
		if len(self.pivot_xyz_realsense_list)>0:
			msg = self.pivot_xyz_realsense_list.pop(0)
			data = msg[0]
			self.pivot_message_realsense_time = msg[1]
			self.pivot_xyz_realsense =  [data.transform.translation.x,
				data.transform.translation.y,
				data.transform.translation.z]

			self.pivot_xyz = self.pivot_xyz_realsense

			self.pivot_xyz_realsense_has_new = True
		else:
			self.pivot_xyz_realsense_has_new = False


	def pivot_xyz_estimated_callback(self,data):
		self.pivot_xyz_estimated_list.append([data,time.time()])
		if len(self.pivot_xyz_estimated_list)>1:
			self.pivot_xyz_estimated_list.pop(0)
		self.data_available_list[self.pivot_xyz_estimated_available_index]=True

	def pivot_xyz_estimated_unpack(self):
		if len(self.pivot_xyz_estimated_list)>0:
			msg = self.pivot_xyz_estimated_list.pop(0)
			data = msg[0]
			self.pivot_message_estimated_time = msg[1]
			self.pivot_xyz_estimated =  [data.transform.translation.x,
				data.transform.translation.y,
				data.transform.translation.z]

			#if there has been no message from the realsense
			if (self.pivot_xyz_realsense is None) or (self.pivot_message_estimated_time-self.pivot_message_realsense_time>.5):
				self.pivot_xyz = self.pivot_xyz_estimated

			self.pivot_xyz_estimated_has_new = True
		else:
			self.pivot_xyz_estimated_has_new = False

	def generalized_positions_callback(self,data):
		self.generalized_positions_list.append(data)
		if len(self.generalized_positions_list)>1:
			self.generalized_positions_list.pop(0)
		self.data_available_list[self.generalized_positions_available_index]=True

	def generalized_positions_unpack(self):
		if len(self.generalized_positions_list)>0:
			data = self.generalized_positions_list.pop(0)
			self.generalized_positions = data.data
			self.state_not_exists_bool = False

			self.generalized_positions_has_new = True
		else:
			self.generalized_positions_has_new = False

	def barrier_func_control_command_callback(self,data):
		self.barrier_func_control_command_list.append(data)
		if len(self.barrier_func_control_command_list)>1:
			self.barrier_func_control_command_list.pop(0)
		self.data_available_list[self.barrier_func_control_command_available_index]=True

	def barrier_func_control_command_unpack(self):
		if len(self.barrier_func_control_command_list)>0:
			data = self.barrier_func_control_command_list.pop(0)
			self.command_msg = pmh.command_stamped_to_command_dict(data)

			self.barrier_func_control_command_has_new = True
		else:
			self.barrier_func_control_command_has_new = False

	def torque_bound_callback(self,data):
		self.torque_bound_list.append(data)
		if len(self.torque_bound_list)>1:
			self.torque_bound_list.pop(0)
		self.data_available_list[self.torque_bound_available_index]=True

	def torque_bound_unpack(self):
		if len(self.torque_bound_list)>0:
			data = self.torque_bound_list.pop(0)
			self.torque_bounds = json.loads(data.data).tolist()

			self.torque_bound_has_new = True
		else:
			self.torque_bound_has_new = False