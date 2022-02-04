import json
import pickle
import numpy as np
import scipy
import os,sys,inspect
from scipy.spatial.transform import Rotation as Rotation_class
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
import copy

def load_shape_data(name_in):
    curr_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(curr_dir)
    gparentdir = os.path.dirname(parentdir)
    # print 'parentdir', parentdir
    fname = os.path.join(gparentdir, 'Modelling', 'shape_description', name_in+".json")
    f = open(fname)
    # f = open("/home/oneills/Documents/panda/base_ws/src/franka_ros_interface/franka_ros_controllers/scripts/models/shape_description/"+name_in+".json")
    shape_data = json.load(f)

    vertex_array = np.array([shape_data["x_vert"],shape_data["y_vert"]])
    apriltag_id = shape_data["apriltag_id"]
    apriltag_pos = shape_data["centroid_to_apriltag"]

    return vertex_array, apriltag_id ,apriltag_pos


def pose_list_to_matrix(pose_in):
	r = Rotation_class.from_quat(pose_in[3:7])
	# np.horzcat[r.as_dcm()
	# rotation_matrix =  r.as_dcm()
	rotation_matrix =  r.as_matrix()
	translation_vector = np.array([pose_in[0:3]])
	return np.vstack([
		np.hstack([rotation_matrix,np.transpose(translation_vector)]),
		np.array([0.0,0.0,0.0,1.0])])

def invert_transform_matrix(mat_in):
	rotation_matrix = mat_in[0:3,0:3]
	translation_vector = mat_in[0:3,3]

	inverse_rotation_matrix = np.transpose(rotation_matrix)
	inverse_translation_vector = np.dot(inverse_rotation_matrix,-translation_vector)

	return np.vstack([
		np.hstack([inverse_rotation_matrix,np.transpose(np.array([inverse_translation_vector]))]),
		np.array([0.0,0.0,0.0,1.0])])

def Kalman_update(x_prev,P_prev,z_current,H,Q,R):
	x_predicted = x_prev
	P_predicted = P_prev + Q

	measurement_residual = z_current - np.dot(H,x_predicted)
	M1 = np.dot(P_predicted,np.transpose(H))
	M2 = R+np.dot(H,np.dot(P_predicted,np.transpose(H)))
	kalman_gain = np.transpose(np.linalg.solve(np.transpose(M2),np.transpose(M1)))
	x_updated = x_predicted + np.dot(kalman_gain,measurement_residual)
	P_updated = np.dot(np.identity(len(x_prev))-np.dot(kalman_gain,H),P_predicted)
	return x_updated,P_updated

def synchronize_messages(data_dict,dt):


	new_data_dict = {}
	index_list = {}
	min_time_list = []
	max_time_list = []

	for message_type in data_dict.keys():
		min_time_list.append(data_dict[message_type][0]['time'])
		max_time_list.append(data_dict[message_type][-1]['time'])
		new_data_dict[message_type] = []
		index_list[message_type] = 0

	min_time = np.max(min_time_list)
	max_time = np.min(max_time_list)

	current_time = min_time

	while current_time<=max_time:
		for message_type in data_dict.keys():

			while data_dict[message_type][index_list[message_type]]['time']<current_time:
				index_list[message_type]+=1
				
			if data_dict[message_type][index_list[message_type]]['time']>current_time:
				new_data_dict[message_type].append(copy.deepcopy(data_dict[message_type][index_list[message_type]-1]))
			else:
				new_data_dict[message_type].append(copy.deepcopy(data_dict[message_type][index_list[message_type]]))

			new_data_dict[message_type][-1]['time']=current_time

		current_time+=dt 
	return new_data_dict	



if __name__ == '__main__':
	my_path = 'C:/Users/taylorott/Dropbox (MIT)/pbal_assets/Experiments\InitialEstimatorDataPlusQPDebug-Jan-2022/'
	fname = '2022-01-21-17-16-17-experiment012-rectangle-no-mass.pickle'
	# fname = '2022-01-21-17-17-32-experiment013-rectangle-no-mass.pickle'
	
	l_contact = .1 
	force_scale = .015

	fig = plt.figure()
	# axs = fig.add_subplot(111,projection = '3d')
	axs = fig.add_subplot(111)

	# print axs

	camera_to_world_homog = pose_list_to_matrix([5.06423387e-01, 6.15130675e-01, 1.47602280e-01, -7.77802963e-03, -7.01780459e-01, 7.12291922e-01, 9.16006536e-03])

	object_vertex_array, apriltag_id ,apriltag_pos = load_shape_data('rectangle')
	object_vertex_array = np.vstack([object_vertex_array,
                            np.zeros(len(object_vertex_array[0])),
                            np.ones(len(object_vertex_array[0]))])

	object_vertex_array = np.hstack([object_vertex_array,np.transpose(np.array([object_vertex_array[:,0]]))])


	hand_points = np.array([[0.0,0.0],
	                        [.0505,-.0505],
	                        [0.0375,0.0375],
	                        [1.0,1.0]])
	hand_midpoint = np.array([0.0,0.0,.0375,1.0])
	hand_tangent = np.array([[0.0],[1.0],[0.0],[0.0]])
	hand_normal = np.array([[1.0],[0.0],[0.0],[0.0]])

	april_tag_pose_marker_frame_homog =  pose_list_to_matrix([-apriltag_pos[0], -apriltag_pos[1], 0,0.0,0.0,0.0,1.0])
	vertex_array_marker_frame = np.dot(april_tag_pose_marker_frame_homog,object_vertex_array)

	


	data_dict = pickle.load(open(my_path+fname, 'rb'))

	data_dict = synchronize_messages(data_dict,dt=.03)

	temp_sum1a = np.zeros([3,3])
	temp_sum2a = np.zeros(3)
	temp_sum1b = np.zeros([3,3])
	temp_sum2b = np.zeros(3)


	for i in range(len(data_dict['ee_pose_in_world_from_franka_publisher'])):
		ee_pose_world =  data_dict['ee_pose_in_world_from_franka_publisher'][i]['msg']
		contact_pose_homog = pose_list_to_matrix(ee_pose_world)
		hand_points_world = np.dot(contact_pose_homog,hand_points)	
		hand_midpoint_world = np.dot(contact_pose_homog,hand_midpoint)

		hand_normal_world = np.dot(contact_pose_homog,hand_normal)
		hand_2D_normal_world = np.array([hand_normal_world[0],hand_normal_world[2]])
		hand_2D_normal_world = hand_2D_normal_world/np.sqrt(np.sum(hand_2D_normal_world ** 2))

		hand_tangent_world = np.dot(contact_pose_homog,hand_tangent)
		hand_2D_tangent_world = np.array([hand_tangent_world[0],hand_tangent_world[2]])
		hand_2D_tangent_world = hand_2D_tangent_world/np.sqrt(np.sum(hand_2D_tangent_world ** 2))		

		z = np.dot(np.array([[hand_midpoint_world[0],hand_midpoint_world[2]]],dtype=np.float_),-hand_2D_normal_world)[0][0]

		temp_mat = np.array([[-hand_2D_normal_world[0,0],-hand_2D_normal_world[1,0],1.0]],dtype=np.float_)
		temp_vec = z*np.array([-hand_2D_normal_world[0,0],-hand_2D_normal_world[1,0],1.0],dtype=np.float_)

		temp_mat =  np.dot(np.transpose(temp_mat),temp_mat)

		if hand_2D_normal_world[0]>.06:
			temp_sum1a = temp_sum1a+temp_mat
			temp_sum2a = temp_sum2a+temp_vec
		if hand_2D_normal_world[0]<-.06:
			temp_sum1b = temp_sum1b+temp_mat
			temp_sum2b = temp_sum2b+temp_vec

	pivot1_info = np.linalg.solve(temp_sum1a,temp_sum2a)
	pivot2_info = np.linalg.solve(temp_sum1b,temp_sum2b)




	d0 = .01
	s0 = 0.0

	
	Q=np.identity(3)
	P=np.identity(3)



	R= 1.0
	Q = .01*Q
	P = 100.0*P

	Pleft = P 
	Pright = P 
	Pboth = P


	ee_pose_world =  data_dict['ee_pose_in_world_from_franka_publisher'][0]['msg']
	contact_pose_homog = pose_list_to_matrix(ee_pose_world)
	hand_points_world = np.dot(contact_pose_homog,hand_points)	
	hand_midpoint_world = np.dot(hand_points_world,np.array([.5,.5]))
	hand_normal_world = np.dot(contact_pose_homog,hand_normal)
	hand_tangent_world = np.dot(contact_pose_homog,hand_tangent)


	pivot_guess0 = np.transpose(hand_normal_world)*d0+hand_midpoint_world +s0*np.transpose(hand_tangent_world)


	X = np.array([pivot_guess0[0][0],pivot_guess0[0][2],d0])


	Xleft = X 
	Xright = X 
	Xboth = X 





	plot_increment = 10
	count = 0
	while  count <len(data_dict['tag_detections']):
		ee_pose_world =  data_dict['ee_pose_in_world_from_franka_publisher'][count]['msg']
		measured_base_wrench_6D = -np.array(data_dict['end_effector_sensor_in_base_frame'][count]['msg'])
		measured_contact_wrench_6D = -np.array(data_dict['end_effector_sensor_in_end_effector_frame'][count]['msg'])

		contact_pose_homog = pose_list_to_matrix(ee_pose_world)

		hand_points_world = np.dot(contact_pose_homog,hand_points)
		hand_midpoint_world = np.dot(contact_pose_homog,hand_midpoint)
		
		hand_normal_world = np.dot(contact_pose_homog,hand_normal)
		hand_2D_normal_world = np.array([hand_normal_world[0],hand_normal_world[2]])
		hand_2D_normal_world = hand_2D_normal_world/np.sqrt(np.sum(hand_2D_normal_world ** 2))

		hand_tangent_world = np.dot(contact_pose_homog,hand_tangent)
		hand_2D_tangent_world = np.array([hand_tangent_world[0],hand_tangent_world[2]])
		hand_2D_tangent_world = hand_2D_tangent_world/np.sqrt(np.sum(hand_2D_tangent_world ** 2))	
		
		tag_camera_frame_homog = pose_list_to_matrix(data_dict['tag_detections'][count]['msg']['position']+data_dict['tag_detections'][count]['msg']['orientation'])
		vertex_array_world_frame = np.dot(camera_to_world_homog,np.dot(tag_camera_frame_homog,vertex_array_marker_frame))
		
		# print vertex_array_world_frame[1,:]
		obj_x_coords = vertex_array_world_frame[0,:]
		obj_y_coords = vertex_array_world_frame[2,:]


		contact_point = measured_contact_wrench_6D[-1]/measured_contact_wrench_6D[0]
		contact_point = np.max([np.min([contact_point,l_contact/2]),-l_contact/2])
		alpha_2 = (contact_point+l_contact/2)/(l_contact)
		alpha_1 = 1-alpha_2
		hand_cop = np.dot(hand_points_world,np.array([alpha_1,alpha_2]))



		z1 = np.dot(np.array([[hand_midpoint_world[0],hand_midpoint_world[2]]],dtype=np.float_),-hand_2D_normal_world)[0][0]
		H1 = np.transpose(np.vstack([-hand_2D_normal_world,1.0]))

		alpha_z2 = .01
		z2 = alpha_z2*((hand_cop[0]*measured_base_wrench_6D[2])-(hand_cop[2]*measured_base_wrench_6D[0]))
		H2 = alpha_z2*np.array([[measured_base_wrench_6D[2],-measured_base_wrench_6D[0],0.0]])

		z = np.array([z1,z2])
		H = np.vstack([H1,H2])

		Xboth,Pboth = Kalman_update(Xboth,Pboth,z,H,Q,R)
		if hand_2D_normal_world[0]<-.03:
			Xleft,Pleft = Kalman_update(Xleft,Pleft,z,H,Q,R)
		if hand_2D_normal_world[0]>.00:
			Xright,Pright = Kalman_update(Xright,Pright,z,H,Q,R)




		if count%plot_increment==0:
			axs.clear()
		
			hand_frame_length = .03
			axs.plot([-1,1],[0,0])
			axs.plot(obj_x_coords, obj_y_coords,color = 'blue')
			axs.plot(hand_points_world[0,:],hand_points_world[2,:],color = 'red')

			# axs.plot([hand_midpoint_world[0],hand_midpoint_world[0]+hand_frame_length*hand_normal_world[0,0]],
			# 		 [hand_midpoint_world[2],hand_midpoint_world[2]+hand_frame_length*hand_normal_world[2,0]],color = 'green')
			
			axs.scatter(Xleft[0],Xleft[1],color = 'red')
			axs.scatter(Xright[0],Xright[1],color = 'red')
			axs.scatter(Xboth[0],Xboth[1],color = 'red')
			axs.scatter(pivot1_info[0],pivot1_info[1],color = 'green')
			axs.scatter(pivot2_info[0],pivot2_info[1],color = 'green')


			axs.plot([hand_cop[0],hand_cop[0]+force_scale*measured_base_wrench_6D[0]],
					[hand_cop[2],hand_cop[2]+force_scale*measured_base_wrench_6D[2]],color = 'green')
			axs.scatter(hand_cop[0],hand_cop[2],color = 'blue')
			
			axs.axis('equal')
			axs.set_ylim([-.5, .3])
			axs.set_xlim([.25, .75])


			plt.pause(0.01)

		count+=1
