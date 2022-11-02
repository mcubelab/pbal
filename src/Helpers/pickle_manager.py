#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

import pickle

import numpy as np

class pickle_manager(object):
    def __init__(self,path):
        self.path = path

    def generate_parse_dict(self):
        global pmh
        import Helpers.pbal_msg_helper as pmh
        self.parse_dict = {
            'WrenchStamped': pmh.wrench_stamped2list,
            'PoseStamped': pmh.pose_stamped2list,
            'TransformStamped': pmh.transform_stamped2list,
            'ControlCommandStamped': pmh.command_stamped_to_command_dict,
            'FrictionParamsStamped': pmh.friction_stamped_to_friction_dict,
            'GeneralizedPositionsStamped': pmh.parse_generalized_positions_stamped,
            'PivotSlidingCommandedFlagStamped': pmh.parse_pivot_sliding_commanded_flag,
            'QPDebugStamped': pmh.qp_debug_stamped_to_qp_debug_dict,
            'SlidingStateStamped': pmh.sliding_stamped_to_sliding_dict,
            'TorqueBoundsStamped': pmh.parse_torque_bounds_stamped,
            'TorqueConeBoundaryFlagStamped': pmh.parse_torque_cone_boundary_flag_stamped,
            'TorqueConeBoundaryTestStamped': pmh.parse_torque_cone_boundary_test_stamped,
            'AprilTagDetectionArray': pmh.parse_apriltag_detection_array,
            'CameraInfo': pmh.parse_camera_info,
        }

    def generate_experiment_name(self,experiment_label=None):
        if experiment_label is None:
            experiment_label = 'test_data'

        experiment_nums = []
        for file in os.listdir(self.path):
            if file.endswith('.bag') or file.endswith('.pickle'):
                fname = os.path.splitext(file)
                fname_tokens = fname[0].split('-')
                experiment_token = [token for token in fname_tokens if 'experiment' in token ]

                if not experiment_token:
                    continue
                experiment_nums.append(int(experiment_token[0][-3:]))

        # new experiment number
        if not experiment_nums:
            new_experiment_num = 1
        else:
            new_experiment_num = np.amax(np.array(experiment_nums, dtype=int)) + 1

        #return experiment name
        return experiment_label+'-'+('experiment{:04d}'.format(new_experiment_num))

    def read_buffer(self,topic_buffer,message_type):
        parse_function = self.parse_dict[message_type]
        time_list = []
        msg_list = []


        for msg in topic_buffer:
            time_list.append(msg.header.stamp.to_sec())
            msg_list.append(parse_function(msg))

        return time_list, msg_list

    def generate_save_dictionary(self,topic_list,buffer_dict,message_type_dict,fname):
        save_dict = {}
        msg_types = []

        for topic in topic_list:
            if message_type_dict[topic] in self.parse_dict and buffer_dict[topic]:
                time_list, msg_list = self.read_buffer(buffer_dict[topic],message_type_dict[topic])
                save_dict[topic]={'time_list':time_list, 'msg_list':msg_list}

            elif message_type_dict[topic] == 'Image' and buffer_dict[topic]:
                save_dict[topic]={'time_list':buffer_dict[topic], 'msg_list':[None]*len(buffer_dict[topic])}

        return save_dict

    def store_in_pickle(self,topic_list,buffer_dict,message_type_dict,experiment_label=None, transform_dict = None):
        self.generate_parse_dict()

        fname = self.generate_experiment_name(experiment_label)
        data = self.generate_save_dictionary(topic_list,buffer_dict,message_type_dict,fname)
        data['transform_dict'] = transform_dict
        print('storing data into: '+fname+'.pickle')
        with open(os.path.join(self.path,fname) + '.pickle', 'wb') as handle:
            pickle.dump(data, handle, protocol=pickle.HIGHEST_PROTOCOL)

