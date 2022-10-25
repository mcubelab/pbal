#!/usr/bin/env python
import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0,os.path.dirname(currentdir))

from cv_bridge import CvBridge
import cv2
import rosbag

def test_rosbag_to_avi(dir_save_bagfile,bagfile_name):
    img_array = []

    fpath = os.path.join(dir_save_bagfile, bagfile_name)
    print('Loading: ' + bagfile_name)

    # build data structure
    bridge = CvBridge()

    num_frames = 0
    # fill out topics
    with rosbag.Bag(fpath, 'r') as bag:
        for topic, msg, time, in bag.read_messages():
            if msg._type == 'sensor_msgs/Image':
                msg = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                height, width, layers = msg.shape
                size = (width, height)
                img_array.append(msg)
                cv2.imshow("Image window", msg)
                cv2.waitKey(3)
                num_frames+=1


    print('num_frames: ',num_frames)
    save_name = dir_save_bagfile+'/'+bagfile_name.split('.')[0]+'.avi'
    print(save_name)
    video_out = cv2.VideoWriter(save_name, cv2.VideoWriter_fourcc(*'DIVX'), 30, size)

    for i in range(len(img_array)):
        video_out.write(img_array[i])
    video_out.release()

def test_read_avi(dir_save_bagfile,avifile_name):
    vidcap = cv2.VideoCapture(dir_save_bagfile+'/'+avifile_name)

    success = True
    num_frames = 0
    while success:
        success,msg = vidcap.read()
        if success:
            num_frames+=1
            cv2.imshow("Image window", msg)
            cv2.waitKey(3)

    print('num_frames: ',num_frames)

if __name__ == "__main__":
    dir_save_bagfile = '/home/thecube/Documents/pbal_experiments/gtsam_test_data'
    file_name = '2022-06-23-00-49-23-test_data-experiment0013'

    test_rosbag_to_avi(dir_save_bagfile,file_name+'.bag')
    # test_read_avi(dir_save_bagfile,file_name+'.avi')

