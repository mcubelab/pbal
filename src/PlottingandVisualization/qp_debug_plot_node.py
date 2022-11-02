#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

from Helpers.ros_manager import ros_manager

import rospy

def plot_increment(my_axis,X,dX,color):
    my_axis.plot([X[0],X[0]+dX[0]],[X[1],X[1]+dX[1]],color)

def plot_constraint(my_axis,offset,normal,color):
    my_axis.plot([offset[0]+100*normal[1],offset[0]-100*normal[1]],
                 [offset[1]-100*normal[0],offset[1]+100*normal[0]],
                 color)

if __name__ == '__main__':
    rm = ros_manager()
    rospy.init_node('qp_debug_plot')
    rm.subscribe_to_list(['/qp_debug_message'])
    rm.wait_for_necessary_data()

    fig, axs = plt.subplots(1,2)

    while not rospy.is_shutdown():
        rm.unpack_all()

        # unpack variables
        measured_wrench = np.array(rm.qp_debug_dict['measured_wrench'])
        delta_wrench = np.array(rm.qp_debug_dict['delta_wrench'])
        delta_wrench_unconstrained = np.array(rm.qp_debug_dict['delta_wrench_unconstrained'])

        constraint_normals = np.array(rm.qp_debug_dict['constraint_normals'])
        constraint_offsets = np.array(rm.qp_debug_dict['constraint_offsets'])
        slacks = rm.qp_debug_dict['slacks']

        proj_vec_list = np.array(rm.qp_debug_dict['proj_vec_list'])
        error_list = rm.qp_debug_dict['error_list']

        # clear axes
        for ax in axs:
            ax.clear()

        for i in range(len(constraint_offsets)):
            constraint_plotted = False

            Aiq = constraint_normals[i]
            biq = constraint_offsets[i]
            slacki = slacks[i]

            #If the constraint lies in the Normal Force x Tangential Force Plane
            #then plot in on the left subplot
            #the left subplot axes are:
            #horizontal left = positive tangential force
            #vertical up = positive normal force
            #and the constraint/wrench coordinates are [Fn,Ft,Tangential]
            #we will plot using (Index1,Index0), and the invert the X axis at the end
            if np.abs(Aiq[2])<.0001:
                constraint_plotted = True
                
                #plot the true constraint in the quadratic program (including the slack offset)
                offset = measured_wrench[[1,0]]+Aiq[[1,0]]*slacki/(Aiq[0]**2+Aiq[1]**2)
                plot_constraint(axs[0],offset,Aiq[[1,0]],'g')

                #plot the original idealized constraint (without the slack offset)
                offset = Aiq[[1,0]]*biq/(Aiq[0]**2+Aiq[1]**2)
                plot_constraint(axs[0],offset,Aiq[[1,0]],'m')

            #If the constraint lies in the Normal Force x Z Torque Plane
            #then plot in on the right axis
            #the right subplot axes are:
            #horizontal right = positive torque
            #vertical up = positive normal force
            #and the constraint/wrench coordinates are [Fn,Ft,Tangential]
            #we will plot using (Index2,Index0)
            if np.abs(Aiq[1])<.0001:
                constraint_plotted = True

                #plot the true constraint in the quadratic program (including the slack offset)
                offset = measured_wrench[[2,0]]+Aiq[[2,0]]*slacki/(Aiq[0]**2+Aiq[2]**2)
                plot_constraint(axs[1],offset,Aiq[[2,0]],'g')

                #plot the original idealized constraint (without the slack offset)
                offset = Aiq[[2,0]]*biq/(Aiq[0]**2+Aiq[2]**2)
                plot_constraint(axs[1],offset,Aiq[[2,0]],'m')

            if not constraint_plotted:
                print 'constraint not plotted: ', Aiq, '  ', biq

        #plot the minimum of each of the cost terms
        for i in range(len(error_list)):
            proj_veci = proj_vec_list[i]
            errori = error_list[i]

            proj_vec_mag_sqr = (proj_veci[0]**2+proj_veci[1]**2+proj_veci[2]**2)
            if proj_vec_mag_sqr>.001:
                alpha = -errori/proj_vec_mag_sqr

                plot_increment(axs[0],measured_wrench[[1,0]],alpha*proj_veci[[1,0]],'r')
                plot_increment(axs[1],measured_wrench[[2,0]],alpha*proj_veci[[2,0]],'r')


        #plot the both the constrained and unconstrained solutions for the impedance target
        plot_increment(axs[0],measured_wrench[[1,0]],delta_wrench_unconstrained[[1,0]],'b')
        plot_increment(axs[0],measured_wrench[[1,0]],delta_wrench[[1,0]],'k')
        axs[0].plot(measured_wrench[1], measured_wrench[0], 'r*')

        axs[0].set_xlim([ 30, -30])
        axs[0].set_ylim([-10, 50])
        axs[0].set_title('Friction Cone')


        plot_increment(axs[1],measured_wrench[[2,0]],delta_wrench_unconstrained[[2,0]],'b')
        plot_increment(axs[1],measured_wrench[[2,0]],delta_wrench[[2,0]],'k')
        axs[1].plot(measured_wrench[2], measured_wrench[0], 'r*')

        axs[1].set_ylim([-10, 50])
        axs[1].set_xlim([-5, 5])
        axs[1].set_title('Torque Cone')
        plt.pause(0.01)





        
