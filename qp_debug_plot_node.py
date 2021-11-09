#!/usr/bin/env python
import rospy
import pdb
import json
import numpy as np
from std_msgs.msg import String

import matplotlib.pyplot as plt
from matplotlib import cm

def qp_debug_message_callback(data):
    global qp_debug_dict
    if data.data != '':
        qp_debug_dict = json.loads(data.data) 

if __name__ == '__main__':

    rospy.init_node("qp_debug_plot")
    rospy.sleep(1.0)

    qp_debug_dict = None

    qp_debug_message_sub = rospy.Subscriber(
       '/qp_debug_message',
        String,
        qp_debug_message_callback)

    print("Waiting for qp debug message")
    while qp_debug_dict is None:
        rospy.sleep(0.1)

    measured_wrench = qp_debug_dict['measured_wrench']

    fig, axs = plt.subplots(1,2)
    axs[0].set_title('Friction Cone')
    axs[0].plot(measured_wrench[1], measured_wrench[0])
    axs[0].set_ylim([-10, 50])
    axs[0].set_xlim([-30, 30])

    axs[1].set_title('Torque Cone')
    axs[1].plot(measured_wrench[2], measured_wrench[0])
    axs[1].set_ylim([-10, 50])
    axs[1].set_xlim([-5, 5])    

    while not rospy.is_shutdown():

        # unpack variables
        measured_wrench = qp_debug_dict['measured_wrench']
        delta_wrench = qp_debug_dict['delta_wrench']
        delta_wrench_unconstrained = qp_debug_dict[
            'delta_wrench_unconstrained']

        constraint_normals = qp_debug_dict['constraint_normals']
        constraint_offsets = qp_debug_dict['constraint_offsets']
        slacks = qp_debug_dict['slacks']

        proj_vec_list = qp_debug_dict['proj_vec_list']
        error_list = qp_debug_dict['error_list']

        print qp_debug_dict['label_list_cnstr']

            
        # clear axes
        for ax in axs:
            ax.clear()

        for i in range(len(constraint_offsets)):
            constraint_plotted = False

            Aiq = constraint_normals[i]
            biq = constraint_offsets[i]
            slacki = slacks[i]

            if np.abs(Aiq[2])<.0001:
                constraint_plotted = True

                alpha = slacki/(Aiq[0]**2+Aiq[1]**2)
                axs[0].plot(
                    [measured_wrench[1]+Aiq[1]*alpha+100.*Aiq[0],measured_wrench[1]+Aiq[1]*alpha-100.*Aiq[0]], 
                    [measured_wrench[0]+Aiq[0]*alpha-100.*Aiq[1],measured_wrench[0]+Aiq[0]*alpha+100.*Aiq[1]], 'g')


                alpha = biq/(Aiq[0]**2+Aiq[1]**2)

                axs[0].plot(
                    [Aiq[1]*alpha+100.*Aiq[0], Aiq[1]*alpha-100.*Aiq[0]], 
                    [Aiq[0]*alpha-100.*Aiq[1], Aiq[0]*alpha+100.*Aiq[1]], 'm')


            if np.abs(Aiq[1])<.0001:
                constraint_plotted = True

                alpha = slacki/(Aiq[0]**2+Aiq[2]**2)
                axs[1].plot(
                    [measured_wrench[2]+Aiq[2]*alpha+100.*Aiq[0],measured_wrench[2]+Aiq[2]*alpha-100.*Aiq[0]], 
                    [measured_wrench[0]+Aiq[0]*alpha-100.*Aiq[2],measured_wrench[0]+Aiq[0]*alpha+100.*Aiq[2]], 'g')

                alpha = biq/(Aiq[0]**2+Aiq[2]**2)

                axs[1].plot(
                    [Aiq[2]*alpha+100.*Aiq[0], Aiq[2]*alpha-100.*Aiq[0]], 
                    [Aiq[0]*alpha-100.*Aiq[2], Aiq[0]*alpha+100.*Aiq[2]], 'm')

            if not constraint_plotted:
                print 'constraint not plotted: ', Aiq, "  ", biq

        for i in range(len(error_list)):
            proj_veci = proj_vec_list[i]
            errori = error_list[i]

            proj_vec_mag_sqr = (proj_veci[0]**2+proj_veci[1]**2+proj_veci[2]**2)
            if proj_vec_mag_sqr>.001:
                alpha = errori/proj_vec_mag_sqr

                axs[0].plot(
                        [measured_wrench[1],measured_wrench[1]+alpha*proj_veci[1]], 
                        [measured_wrench[0],measured_wrench[0]+alpha*proj_veci[0]], 'r')
                axs[1].plot(
                        [measured_wrench[2],measured_wrench[2]+alpha*proj_veci[2]], 
                        [measured_wrench[0],measured_wrench[0]+alpha*proj_veci[0]], 'r')



        axs[0].plot(
            [measured_wrench[1], measured_wrench[1] + delta_wrench_unconstrained[1]], 
            [measured_wrench[0], measured_wrench[0] + delta_wrench_unconstrained[0]], 'b')
        
        axs[0].plot(
            [measured_wrench[1], measured_wrench[1] + delta_wrench[1]], 
            [measured_wrench[0], measured_wrench[0] + delta_wrench[0]], 'k')

        axs[0].plot(measured_wrench[1], measured_wrench[0], 'r*')

        axs[0].set_xlim([-30, 30])
        axs[0].set_ylim([-10, 50])
        axs[0].set_title('Friction Cone')

        axs[1].plot(
            [measured_wrench[2], measured_wrench[2] + delta_wrench_unconstrained[2]], 
            [measured_wrench[0], measured_wrench[0] + delta_wrench_unconstrained[0]], 'b')
        axs[1].plot(
            [measured_wrench[2], measured_wrench[2] + delta_wrench[2]], 
            [measured_wrench[0], measured_wrench[0] + delta_wrench[0]], 'k')

        axs[1].plot(measured_wrench[2], measured_wrench[0], 'r*')

        axs[1].set_ylim([-10, 50])
        axs[1].set_xlim([-5, 5])
        axs[1].set_title('Torque Cone')
        # plt.show()
        plt.pause(0.01)





        
