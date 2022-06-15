#!/usr/bin/python


import csv
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

########################################################################################################################
###################################### Grab data from input CSV file ###################################################
########################################################################################################################

df = pd.read_csv('/home/lin/develop/ros/soslab_ws/src/slam/msckf_dvio/test_result/msckf_updater_both.csv')
# df = pd.read_csv('/home/lin/develop/ros/soslab_ws/src/slam/msckf_dvio/test_result/msckf_updater_R_T.csv')
# df = pd.read_csv('/home/lin/develop/ros/soslab_ws/src/slam/msckf_dvio/test_result/msckf_updater_R.csv')


########################################################################################################################
###################################### Plot data from input CSV file ###################################################
########################################################################################################################

# plot
df.plot(x="count", y=["v_m_y_rt", "v_m_y_r"])
plt.show()
