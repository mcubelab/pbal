#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped, TransformStamped
from std_msgs.msg import Bool, Int32
from pbal.msg import SlidingStateStamped, FrictionParamsStamped
import ros_helper as rh
import pbal_msg_helper as pmh
import time
import numpy as np
from Estimation import friction_reasoning
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
			self.panda_hand_in_base_pose = None 
			self.base_z_in_panda_hand = None

			panda_hand_in_base_pose_sub = rospy.Subscriber(
				topic, 
				PoseStamped, 
				self.ee_pose_callback, 
				queue_size=1)

		elif topic == '/end_effector_sensor_in_end_effector_frame':
			self.unpack_list.append(self.end_effector_wrench_unpack)
			self.measured_contact_wrench_list = []
			self.measured_contact_wrench_available_index = len(self.data_available_list)-1
			self.measured_contact_wrench = None

			self.end_effector_wrench_sub = rospy.Subscriber(
				topic, 
				WrenchStamped,  
				self.end_effector_wrench_callback)

		elif topic == '/end_effector_sensor_in_base_frame':
			self.unpack_list.append(self.end_effector_wrench_base_frame_unpack)
			self.measured_base_wrench_list = []
			self.measured_base_wrench_available_index = len(self.data_available_list)-1
			self.measured_base_wrench = None

			self.end_effector_wrench_base_frame_sub = rospy.Subscriber(
				topic, 
				WrenchStamped,  
				self.end_effector_wrench_base_frame_callback)

		elif topic == '/friction_parameters':
			self.unpack_list.append(self.friction_parameter_unpack)
			self.friction_parameter_list = []
			self.friction_parameter_available_index = len(self.data_available_list)-1
			self.friction_parameter_dict, dummy1 ,dummy2 = friction_reasoning.initialize_friction_dictionaries()
			self.friction_parameter_sub = rospy.Subscriber(
				'/friction_parameters', 
				FrictionParamsStamped, 
				self.friction_parameter_callback)

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