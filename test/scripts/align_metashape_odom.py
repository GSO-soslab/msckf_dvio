#!/usr/bin/python



import argparse

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from numpy.linalg import inv

import rosbag
import rospy
import tf
from nav_msgs.msg import Odometry


#### Usage: evaluate ground truth odometry with MSCKF odometry
####  Ground Truth: metashape mono camera odometry
####  MSCKF: IMU, DVL, Pressure

# command:
# python3 /home/lin/develop/ros/soslab_ws/src/slam/msckf_dvio/test/scripts/align_metashape_odom.py \
# output.bag glrc_3_13_odom_compare_test1.bag -v 


def main():

  parser = argparse.ArgumentParser(description='Evaluate the odometry.')
  parser.add_argument('outputbag',
                      help='output bag file with odometry aligned')
  parser.add_argument('inputbag',
                      help='input bag to compare odometry')
  parser.add_argument('-v', '--verbose', action="store_true", default=False,
                      help='verbose output')                      

  args = parser.parse_args()

  if (args.verbose):
      print("Input bag file: " + args.inputbag)
      print("Writing bag file: " + args.outputbag)

############################################################################################################
###################################### Preapre odom data ###################################################
############################################################################################################

### filter useless cam pose, but left one older then MSCKF INIT timestamp

### get data
  cam_msg = []

  try:
    for topic, msg, t in rosbag.Bag(args.inputbag).read_messages():
      if (topic == "/metashape_cam_odom"):
        cam_msg.append(msg)
        
  finally:
    print ("Finish get data from rosbag with size=%d" % len(cam_msg))
    
############################################################################################################
###################################### Preapre odom data ###################################################
############################################################################################################

  cam_msg_new = []
  outbag = rosbag.Bag(args.outputbag, 'w')

  ### get transformation of camera init frame with repect to anchor frame
  T_C0_Ci = tf.transformations.quaternion_matrix([
                cam_msg[0].pose.pose.orientation.w,
                cam_msg[0].pose.pose.orientation.x,
                cam_msg[0].pose.pose.orientation.y,
                cam_msg[0].pose.pose.orientation.z])
  R_C0_Ci = np.array([[T_C0_Ci[0][0], T_C0_Ci[0][1], T_C0_Ci[0][2]], 
                      [T_C0_Ci[1][0], T_C0_Ci[1][1], T_C0_Ci[1][2]],
                      [T_C0_Ci[2][0], T_C0_Ci[2][1], T_C0_Ci[2][2]]])
  p_C0_Ci = np.array([[cam_msg[0].pose.pose.position.x], 
                      [cam_msg[0].pose.pose.position.y],
                      [cam_msg[0].pose.pose.position.z]])

  ### transform cam pose into init frame
  for i in range(len(cam_msg)):

    ### get each camera transformation (w.r.t. Anchor frame: the first camera)

    T_C0_Cn =  tf.transformations.quaternion_matrix([
            cam_msg[i].pose.pose.orientation.w,
            cam_msg[i].pose.pose.orientation.x,
            cam_msg[i].pose.pose.orientation.y,
            cam_msg[i].pose.pose.orientation.z])

    R_C0_Cn = np.array([[T_C0_Cn[0][0], T_C0_Cn[0][1], T_C0_Cn[0][2]], 
                        [T_C0_Cn[1][0], T_C0_Cn[1][1], T_C0_Cn[1][2]],
                        [T_C0_Cn[2][0], T_C0_Cn[2][1], T_C0_Cn[2][2]]])
    p_C0_Cn = np.array([[cam_msg[i].pose.pose.position.x], 
                        [cam_msg[i].pose.pose.position.y],
                        [cam_msg[i].pose.pose.position.z]])

    ### tranform to camera init frame

    # R_Ci_Cn = R_C0_Ci^T * R_C0_Cn
    R_Ci_Cn = np.matmul(inv(R_C0_Ci),R_C0_Cn)
    # p_Ci_Cn = R_C0_Ci^T * (p_C0_Cn - p_C0_Ci)
    p_Ci_Cn = np.matmul(inv(R_C0_Ci),p_C0_Cn-p_C0_Ci)


    ### transform to IMU frame

    R_Ci_Ii = np.array([[0.00786236726823823, 0.9999532121420647,   0.0056353090162310805],
                        [0.9999681294260654, -0.00785441912633285, -0.0014311646745676972],
                        [-0.0013868356345183388, 0.005646381757991208,-0.9999830973871349]])
    p_Ci_Ii = np.array([[-0.1232206727449017],
                        [-0.310185995908388],
                        [-0.27812151547998104]])

    # R_Ii_Cn = R_Ci_Ii^T * R_Ci_Cn
    R_Ii_Cn = np.matmul(inv(R_Ci_Ii),R_Ci_Cn)

    # p_Ii_Cn = R_Ci_Ii^T * (p_Ci_Cn - p_Ci_Ii)
    p_Ii_Cn = np.matmul(inv(R_Ci_Ii),p_Ci_Cn-p_Ci_Ii)

    ### convert into ROS/Odometry msg
    
    msg = Odometry()
    msg.header.frame_id = "odom"
    msg.header.stamp = cam_msg[i].header.stamp
    msg.child_frame_id = "cam"
    msg.pose.pose.position.x = p_Ii_Cn[0][0]
    msg.pose.pose.position.y = p_Ii_Cn[1][0]
    msg.pose.pose.position.z = p_Ii_Cn[2][0]
    # tf bug, it require T-SE(3), but only use R-SO(3)
    T_Ci_Ii = np.identity(4)
    T_Ci_Ii[0:3,0:3] = R_Ii_Cn
    q = tf.transformations.quaternion_from_matrix(T_Ci_Ii)
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]

    # write into rosbag 
    outbag.write("/test_odom", msg, msg.header.stamp)


    # if i==1:
    #   break
  outbag.close()
  print("finsih save rosbag!")


if __name__ == "__main__":
    main()