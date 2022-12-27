# !/usr/bin/env python


#################################################
###### Usage
# python3 /home/lin/develop/ros/soslab_ws/src/slam/msckf_dvio/test/scripts/plot_bias.py \
# -p1 /home/lin/Desktop/temp/imu_bias/2022-12-09-12-59-48.bag \
# -p2 /home/lin/Desktop/temp/imu_bias/2022-12-09-13-21-59.bag -v

import argparse
import rosbag
import rospy
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams['figure.figsize'] = [10, 6]

class evaluateAlgorithm:

    def __init__(self):
        self.path_1 = ''
        self.path_2 = ''

    def main(self):

      self.parse_arguments()

      self.evaluate()

    ### Parse arguments
    def parse_arguments(self):
        parser = argparse.ArgumentParser(description='plot IMU bais.')
        parser.add_argument('-p1','--path1', help='rosbag 1 path')     
        parser.add_argument('-p2','--path2', help='rosbag 2 path')     
        parser.add_argument('-v', '--verbose', action="store_true", default=False,
                            help='verbose output')


        args = parser.parse_args()

        if (args.verbose):
            print('')
            print('============================================================')
            print('Args Info:')
            print("rosbag path 1: %s" % args.path1)
            print("rosbag path 2: %s" % args.path2)
            print('')

        ### set to global value
        self.path_1 = args.path1
        self.path_2 = args.path2

    ### Do the Evaluation
    def evaluate(self):
        
        bag1 = rosbag.Bag(self.path_1)
        bag2 = rosbag.Bag(self.path_2)

        ####### Get the initial time #######
        time1 = 0.0
        time2 = 0.0

        for topic, msg, t in bag1.read_messages("/imu_bias"):
            time1 = msg.header.stamp.to_sec()
            break

        for topic, msg, t in bag2.read_messages("/imu_bias"):
            time2 = msg.header.stamp.to_sec()
            break
    
        init_time = 0.0
        if time1 >= time2:
            init_time = time2
        else:
            init_time = time1

        ####### Get the rosbag 1 data #######
        ## parse IMU bias
        bias_1 = [[],[],[]]
        timestamp_1 = []
        count = 0
        for topic, msg, t in bag1.read_messages("/imu_bias"):

            x = msg.twist.angular.x
            y = msg.twist.angular.y
            z = msg.twist.angular.z
            delta_t = msg.header.stamp.to_sec() - init_time

            bias_1[0].append(x)
            bias_1[1].append(y)
            bias_1[2].append(z)
            timestamp_1.append(delta_t)

            count += 1

        print("bag 1 count: %d" % count)

        ####### Get the rosbag 2 data #######
        ## parse IMU bias
        bias_2 = [[],[],[]]
        timestamp_2 = []
        count = 0
        for topic, msg, t in bag2.read_messages("/imu_bias"):

            x = msg.twist.angular.x
            y = msg.twist.angular.y
            z = msg.twist.angular.z
            delta_t = msg.header.stamp.to_sec() - init_time

            bias_2[0].append(x)
            bias_2[1].append(y)
            bias_2[2].append(z)
            timestamp_2.append(delta_t)

            count += 1

        print("bag 2 count: %d" % count)


        ## TEST:
        time=[]
        data=[[],[],[]]
        for i in range(10):
            time.append(i)
            data[0].append(1)
            data[1].append(2)
            data[2].append(3)

        # ## plot
        fig = plt.figure(1)
        ax = fig.add_subplot(111)
        plot_1= ax.scatter(timestamp_1, bias_1[0], color="cyan", label='Noise=0.1', s=10)
        plot_2 = ax.scatter(timestamp_2, bias_2[0], color="red", label="Noise=0.01", s=10)
        ax.locator_params(nbins=6)
        ax.set_xlabel('timestep [s]', fontsize=15)
        ax.set_ylabel('bias', fontsize=15)
        ax.legend(fontsize=15)
        ax.tick_params(axis='both', which='major', labelsize=15)
        plt.grid()
        plt.title("Bias_X", fontweight='bold', fontsize=24)
        plt.tight_layout()
        plt.savefig('filename_x.png', dpi=300)

        fig = plt.figure(2)
        ax = fig.add_subplot(111)
        plot_1= ax.scatter(timestamp_1, bias_1[1], color="cyan", label='Noise=0.1', s=10)
        plot_2 = ax.scatter(timestamp_2, bias_2[1], color="red", label="Noise=0.01", s=10)
        ax.locator_params(nbins=6)
        ax.set_xlabel('timestep [s]', fontsize=15)
        ax.set_ylabel('bias', fontsize=15)
        ax.legend(fontsize=15)
        ax.tick_params(axis='both', which='major', labelsize=15)
        plt.grid()
        plt.title("Bias_Y", fontweight='bold', fontsize=24)
        plt.tight_layout()
        plt.savefig('filename_y.png', dpi=300)

        fig = plt.figure(3)
        ax = fig.add_subplot(111)
        plot_1= ax.scatter(timestamp_1, bias_1[2], color="cyan", label='Noise=0.1', s=10)
        plot_2 = ax.scatter(timestamp_2, bias_2[2], color="red", label="Noise=0.01", s=10)
        ax.locator_params(nbins=6)
        ax.set_xlabel('timestep [s]', fontsize=15)
        ax.set_ylabel('bias', fontsize=15)
        ax.legend(fontsize=15)
        ax.tick_params(axis='both', which='major', labelsize=15)
        plt.grid()
        plt.title("Bias_Z", fontweight='bold', fontsize=24)
        plt.tight_layout()
        plt.savefig('filename_z.png', dpi=300)


        plt.show()

if __name__ == '__main__':
   object = evaluateAlgorithm()
   object.main()