#!/usr/bin/python
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from numpy.linalg import inv

import rosbag
import rospy
import tf
from nav_msgs.msg import Odometry

########################################################################################################################
#### Usage: parse Metashape Omega-Phi-Kappa text file
#### 1) remove all the header file 
########################################################################################################################



###################################### Grab data from input CSV file ###################################################

### get data using pandas
df = pd.read_csv("/home/lin/develop/data/underIce/GLRC/day_3_13/left_cam_pose_metashape.txt", 
                 sep="\t", header=None, 
                 names=["PhotoID", "X", "Y", "Z", "Omega", "Phi", "Kappa",
                        "r11","r12","r13","r21","r22","r23","r31","r32","r33"])

### get each column
t   =  list(df["PhotoID"])
x   =  list(df["X"])
y   =  list(df["Y"])
z   =  list(df["Z"])
r11 =  list(df["r11"])
r12 =  list(df["r12"])
r13 =  list(df["r13"])
r21 =  list(df["r21"])
r22 =  list(df["r22"])
r23 =  list(df["r23"])
r31 =  list(df["r31"])
r32 =  list(df["r32"])
r33 =  list(df["r33"])

############################# Transform all the camera pose into first camera frame ######################################

### get first Transformation between Local frame and Camera_1
T_L_C1 = np.array([[r11[0], r12[0], r13[0], x[0]], 
                   [r21[0], r22[0], r23[0], y[0]],
                   [r31[0], r32[0], r33[0], z[0]],
                   [0, 0, 0, 1.0]])
T_C1_L = inv(T_L_C1)              
# print(T_L_C1)
# print(T_C1_L)

### check data size
lists = [t,x,y,z,r11,r12,r13,r21,r22,r23,r31,r32,r33]
it = iter(lists)
the_len = len(next(it))
if not all(len(l) == the_len for l in it):
     raise ValueError('not all data have same length!')

### save camera pose into ros bag
outbag = rosbag.Bag("/home/lin/develop/data/underIce/GLRC/day_3_13/metashape_cam_pose.bag", 'w')
print("working on the parsing ......")
print()

for i in range(len(t)):
       ### transfrom all the camera into first camera frame
       T_L_Ci = np.array([[r11[i], r12[i], r13[i], x[i]], 
                          [r21[i], r22[i], r23[i], y[i]],
                          [r31[i], r32[i], r33[i], z[i]],
                          [0, 0, 0, 1.0]])
       T_C1_Ci = np.matmul(T_C1_L,T_L_Ci)
       # R_C1_Ci = T_C1_Ci[0:3,0:3]

       ### convert transformation into ROS/Odometry msg
       msg = Odometry()
       msg.header.frame_id = "cam_1"
       msg.header.stamp = rospy.Time.from_sec(t[i])
       msg.child_frame_id = "cam_i"
       msg.pose.pose.position.x = T_C1_Ci[0][3]
       msg.pose.pose.position.y = T_C1_Ci[1][3]
       msg.pose.pose.position.z = T_C1_Ci[2][3]
       # tf bug, it require T-SE(3), but only use R-SO(3)
       q = tf.transformations.quaternion_from_matrix(T_C1_Ci)
       msg.pose.pose.orientation.x = q[0]
       msg.pose.pose.orientation.y = q[1]
       msg.pose.pose.orientation.z = q[2]
       msg.pose.pose.orientation.w = q[3]

       ### write into ros bag
       outbag.write("metashape_cam_pose", msg, msg.header.stamp)

print("finsih save rosbag!")
outbag.close()

########################################################################################################################
###################################### Plot data for test ###################################################
########################################################################################################################

# ### plot
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# position = ax.scatter(x, y, z)

# plt.tight_layout()
# plt.show()


# ### get a object
# # dictionaryObject = df.to_dict();
# # print(dictionaryObject)

