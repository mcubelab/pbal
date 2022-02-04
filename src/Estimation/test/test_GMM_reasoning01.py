import json
import pickle
import numpy as np
import scipy
import os,sys,inspect
from scipy.spatial.transform import Rotation as Rotation_class
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm

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
	rotation_matrix =  r.as_dcm()
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
	P_updated = np.dot(np.identity(4)-np.dot(kalman_gain,H),P_predicted)
	return x_updated,P_updated


if __name__ == '__main__':
	my_path = 'C:/Users/taylorott/Dropbox (MIT)/pbal_assets/Experiments\InitialEstimatorDataPlusQPDebug-Jan-2022/'
	fname = '2022-01-21-17-16-17-experiment012-rectangle-no-mass.pickle'
	# fname = '2022-01-21-17-17-32-experiment013-rectangle-no-mass.pickle'
	
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

	hand_normal_offset = -.01
	hand_points = np.array([[hand_normal_offset]*2,
	                        [.0505,-.0505],
	                        [0.0375,0.0375],
	                        [1.0,1.0]])
	hand_tangent = np.array([[0.0],[1.0],[0.0],[0.0]])
	hand_normal = np.array([[1.0],[0.0],[0.0],[0.0]])

	april_tag_pose_marker_frame_homog =  pose_list_to_matrix([-apriltag_pos[0], -apriltag_pos[1], 0,0.0,0.0,0.0,1.0])
	vertex_array_marker_frame = np.dot(april_tag_pose_marker_frame_homog,object_vertex_array)




	data_dict = pickle.load(open(my_path+fname, 'rb'))



	d0 = .18
	s0 = -.06

	R=.0001 
	Q=50000.0*np.identity(4)
	P=5000.0*np.identity(4)

	ee_pose_world =  data_dict['ee_pose_in_world_from_franka_publisher'][0]['msg']
	contact_pose_homog = pose_list_to_matrix(ee_pose_world)
	hand_points_world = np.dot(contact_pose_homog,hand_points)	
	hand_midpoint_world = np.dot(hand_points_world,np.array([.5,.5]))
	hand_normal_world = np.dot(contact_pose_homog,hand_normal)
	hand_tangent_world = np.dot(contact_pose_homog,hand_tangent)


	pivot_guess0 = np.transpose(hand_normal_world)*d0+hand_midpoint_world +s0*np.transpose(hand_tangent_world)

	X = np.array([pivot_guess0[0][0],pivot_guess0[0][2],d0,s0])



	pivot_array = []

	x_scatter_array = []
	y_scatter_array = []
	z_scatter_array = []


	temp_sum1a = np.zeros([3,3])
	temp_sum2a = np.zeros(3)
	temp_sum1b = np.zeros([3,3])
	temp_sum2b = np.zeros(3)


	num_data1 = 0
	num_data2 = 0
	for i in range(len(data_dict['ee_pose_in_world_from_franka_publisher'])):
		ee_pose_world =  data_dict['ee_pose_in_world_from_franka_publisher'][i]['msg']
		contact_pose_homog = pose_list_to_matrix(ee_pose_world)
		hand_points_world = np.dot(contact_pose_homog,hand_points)	
		hand_midpoint_world = np.dot(hand_points_world,np.array([.5,.5]))

		hand_normal_world = np.dot(contact_pose_homog,hand_normal)
		hand_2D_normal_world = np.array([hand_normal_world[0],hand_normal_world[2]])
		hand_2D_normal_world = hand_2D_normal_world/np.sqrt(np.sum(hand_2D_normal_world ** 2))

		hand_tangent_world = np.dot(contact_pose_homog,hand_tangent)
		hand_2D_tangent_world = np.array([hand_tangent_world[0],hand_tangent_world[2]])
		hand_2D_tangent_world = hand_2D_tangent_world/np.sqrt(np.sum(hand_2D_tangent_world ** 2))		

		z1 = np.dot(np.array([[hand_midpoint_world[0],hand_midpoint_world[2]]]),-hand_2D_normal_world)[0][0]
		z2 = np.dot(np.array([[hand_midpoint_world[0],hand_midpoint_world[2]]]),-hand_2D_tangent_world)[0][0]

		z=np.array([z1,z2])

		H1 = np.transpose(np.vstack([-hand_2D_normal_world,1.0,0.0]))
		H2 = np.transpose(np.vstack([-hand_2D_tangent_world,0.0,1.0]))

		H=np.vstack([H1,H2])

		X,P = Kalman_update(X,P,z,H,Q,R)
		# print 'X', X 
		# print 'P', P
		pivot_array.append(X)



		x_scatter_array.append(-hand_2D_normal_world[0,0])
		y_scatter_array.append(-hand_2D_normal_world[1,0])
		z_scatter_array.append(z1)

		temp_mat = np.array([[-hand_2D_normal_world[0,0],-hand_2D_normal_world[1,0],1.0]])
		temp_vec = z1*np.array([-hand_2D_normal_world[0,0],-hand_2D_normal_world[1,0],1.0])

		temp_mat =  np.dot(np.transpose(temp_mat),temp_mat)

		if hand_2D_normal_world[0]>.06:
			temp_sum1a = temp_sum1a+temp_mat
			temp_sum2a = temp_sum2a+temp_vec
			num_data1+=1
		if hand_2D_normal_world[0]<-.06:
			temp_sum1b = temp_sum1b+temp_mat
			temp_sum2b = temp_sum2b+temp_vec
			num_data2+=1

	pivot1_info = np.linalg.solve(temp_sum1a,temp_sum2a)
	pivot2_info = np.linalg.solve(temp_sum1b,temp_sum2b)

	print temp_sum1a
	print temp_sum1b
	print temp_sum2a
	print temp_sum2b
	print pivot1_info
	print pivot2_info
	print num_data1,num_data2

	# axs.scatter(x_scatter_array,y_scatter_array,z_scatter_array)
	# axs.scatter(x_scatter_array,z_scatter_array)
	# plt.show()

	plot_increment = 10

	count_camera = 0
	count_hand = 0
	while  count_camera <len(data_dict['tag_detections']):

		while data_dict['ee_pose_in_world_from_franka_publisher'][count_hand]['time'] < data_dict['tag_detections'][count_camera]['time']:
			count_hand+=1

		ee_pose_world =  data_dict['ee_pose_in_world_from_franka_publisher'][count_hand]['msg']
		contact_pose_homog = pose_list_to_matrix(ee_pose_world)

		hand_points_world = np.dot(contact_pose_homog,hand_points)
		hand_midpoint_world = np.dot(hand_points_world,np.array([.5,.5]))
		hand_normal_world = np.dot(contact_pose_homog,hand_normal)
		hand_tangent_world = np.dot(contact_pose_homog,hand_tangent)

		tag_camera_frame_homog = pose_list_to_matrix(data_dict['tag_detections'][count_camera]['msg']['position']+data_dict['tag_detections'][count_camera]['msg']['orientation'])
		vertex_array_world_frame = np.dot(camera_to_world_homog,np.dot(tag_camera_frame_homog,vertex_array_marker_frame))
		# print vertex_array_world_frame[1,:]
		obj_x_coords = vertex_array_world_frame[0,:]
		obj_y_coords = vertex_array_world_frame[2,:]

		axs.clear()

		hand_frame_length = .03
		axs.plot([-1,1],[0,0])
		axs.plot(obj_x_coords, obj_y_coords,color = 'blue')
		axs.plot(hand_points_world[0,:],hand_points_world[2,:],color = 'red')
		axs.plot([hand_midpoint_world[0],hand_midpoint_world[0]+hand_frame_length*hand_normal_world[0]],
				 [hand_midpoint_world[2],hand_midpoint_world[2]+hand_frame_length*hand_normal_world[2]],color = 'green')
		axs.scatter(pivot_array[count_hand][0],pivot_array[count_hand][1],color = 'red')
		axs.scatter(pivot1_info[0],pivot1_info[1],color = 'green')
		axs.scatter(pivot2_info[0],pivot2_info[1],color = 'green')
		
		
		
		axs.set_ylim([-.2, .3])
		axs.set_xlim([.25, .75])


		plt.pause(0.01)

		count_camera+=plot_increment
