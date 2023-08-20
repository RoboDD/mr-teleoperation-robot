#! /usr/bin/python

import rospy
from sensor_msgs.msg import Image, JointState
from mycobot_communication.msg import MycobotCoords
# from cv_bridge import CvBridge, CvBridgeError # For python3, no use
import cv2
import os
import csv
import argparse
import sys
import numpy as np
import math
# from message_filters import ApproximateTimeSynchronizer, Subscriber

global data_path
# global experiment_path
# global images_path_fpv
# global images_path_tpv

global WRITE_FLAG
global RECORD_FREQUENCY
global FREQUENCY_10HZ

RECORD_FREQUENCY = 1 # 1Hz
FREQUENCY_10HZ = 0.1 # 10Hz
WRITE_FLAG = True # True: write file in folder, False: not write any file

"""
    Provides conversions between OpenCV and ROS image formats in a hard-coded way.  
    CV_Bridge, the module usually responsible for doing this, is not compatible with Python 3,
     - the language this all is written in.  So we create this module, and all is... well, all is not well,
     - but all works.  :-/
"""
def imgmsg_to_cv2(img_msg):
    # if img_msg.encoding != "bgr8":
    #     rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
        # image_opencv = image_opencv.byteswap().newbyteorder()
    image_opencv = image_opencv[:,:,[2,1,0]]
    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

# Instantiate CvBridge Not competiable with Python3 and ROS noetic!
# bridge = CvBridge()

def image_callback_fpv(msg):
    timestamp = math.floor(rospy.Time.now().to_sec())
    print("Received an fpv image!")

    # Convert your ROS Image message to OpenCV2
    cv2_img = imgmsg_to_cv2(msg)

    
    # path = 'ziniu_data/John/VR_experiment/images/fpv/'
    filename = str(timestamp) + '_savedImage.jpg'

    if WRITE_FLAG == True:
        cv2.imwrite(str(images_path_fpv)+filename, cv2_img)
    # cv2.waitKey(0)
    rospy.sleep(RECORD_FREQUENCY) # wait one second


def image_callback_tpv(msg):
    
    timestamp = math.floor(rospy.Time.now().to_sec())

    print("Received an tpv image!")

    # Convert your ROS Image message to OpenCV2
    cv2_img = imgmsg_to_cv2(msg)

    # path = 'ziniu_data/John/VR_experiment/images/tpv/'
    filename = str(timestamp) + '_savedImage.jpg'

    if WRITE_FLAG == True:
        cv2.imwrite(str(images_path_tpv)+filename, cv2_img)
    # cv2.waitKey(0)
    rospy.sleep(RECORD_FREQUENCY) # wait one second


def joint_callback(msg):
    
    # timestamp = math.floor(rospy.Time.now().to_sec())
    timestamp = rospy.Time.now().to_sec()
    print("Received an joint states!")

    joint_commands = msg.position
    datapoint = list(joint_commands)


    if WRITE_FLAG == True:

        csv_file = open(os.path.join(experiment_path, 'joint_states.csv'), 'a') # 'a' means append row into file
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([timestamp] + datapoint)
        csv_file.close()

    rospy.sleep(FREQUENCY_10HZ)

def pos_callback(msg):
    
    timestamp = rospy.Time.now().to_sec()
    print("Received an positions!")

    positions = msg
    print(positions)
    datapoint = [positions.x, positions.y, positions.z, positions.rx, positions.ry, positions.rz]

    if WRITE_FLAG == True:

        csv_file2 = open(os.path.join(experiment_path, 'real_position.csv'), 'a') # 'a' means append row into file
        csv_writer2 = csv.writer(csv_file2)
        csv_writer2.writerow([timestamp] + datapoint)
        csv_file2.close()

    rospy.sleep(FREQUENCY_10HZ)  

def run(participant_name, experiment_type, data_path):
    # global data_path
    global experiment_path
    global images_path_fpv
    global images_path_tpv

    if WRITE_FLAG == True:
        # Create the participant folder if it does not exist
        data_path = os.path.join(data_path, participant_name)
        if not os.path.exists(data_path):
            os.makedirs(data_path)

        # Create the experiment type (VR or MR) folder within the participant folder
        experiment_path = os.path.join(data_path, experiment_type)
        if not os.path.exists(experiment_path):
            os.makedirs(experiment_path)

        # Create the images folder within the experiment type folder
        images_path_fpv = os.path.join(experiment_path, 'images_fpv/')
        if not os.path.exists(images_path_fpv):
            os.makedirs(images_path_fpv)

        images_path_tpv = os.path.join(experiment_path, 'images_tpv/')
        if not os.path.exists(images_path_tpv):
            os.makedirs(images_path_tpv)

        # Create a CSV file for joint commands
        csv_path = os.path.join(experiment_path, 'joint_states.csv')
        csv_file = open(csv_path, 'w')
        csv_writer = csv.writer(csv_file)
        csv_file.close()

        # Create a CSV file for joint commands
        csv_path2 = os.path.join(experiment_path, 'real_position.csv')
        csv_file2 = open(csv_path2, 'w')
        csv_writer2 = csv.writer(csv_file2)
        csv_file2.close()

    # Init ROS node
    rospy.init_node('data_recorder_node', anonymous=True)

    # rate = rospy.Rate(10)

    # Define your image topic
    image_topic_fpv = "/camera/color/image_raw"
    image_topic_tpv = "/usb_cam/image_raw"
    joint_topic = 'joint_states'
    real_pos_sub = '/mycobot/coords_real'

    print("Recorder ready! Press enter to START! ctrl + c to STOP!")
    input(" -> ") #python3 (ROS Noetic)
    # raw_input(" -> ") #python27 (ROS Melodic)
    print("Start recording")

    rospy.Subscriber(image_topic_fpv, Image, image_callback_fpv) # 4.7hz
    rospy.Subscriber(image_topic_tpv, Image, image_callback_tpv) # 9.9hz
    rospy.Subscriber(joint_topic, JointState, joint_callback) # 15hz
    rospy.Subscriber(real_pos_sub, MycobotCoords, pos_callback)

    # while not rospy.is_shutdown():
    #     # rospy.spinOnce()
    #     # Set up your subscriber and define its callback
    #     rate.sleep()
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Data Recorder Node')
    parser.add_argument('--participant', type=str, help='Participant name')
    parser.add_argument('--experiments', type=str, choices=['VR', 'MR','Touch'], help='Type of experiment (Touch, VR or MR)')
    parser.add_argument('--data-path', type=str, help='Path to save the data')
    # parser.add_argument('--image-topic', type=str, help='Image topic name')
    args = parser.parse_args()

    print("Recorder initialised!")
    run(args.participant, args.experiments, args.data_path)