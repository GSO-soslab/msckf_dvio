# MSCKF-DVIO

**Tightly-coupled Visual-DVL-Inertial Odometry for Robot-based Ice-water Boundary Exploration**

This is the repo for MSCKF-DVIO, a multi-sensor fused odometry using IMU, DVL, Pressure Sensor and Mono camera. The implementation is based on the state-of-the art [MSCKF](https://ieeexplore.ieee.org/document/4209642) framwork and tested in our underice dataset.

<p align="center">
  <img src="./media/glrc_3_13.gif" width="60%" />
</p>

## Installation: 

### Dependencies: 
  - ROS noetic
  - OpenCV 4.2: `sudo apt install libopencv-dev python3-opencv` or build from source
  - Boost: `sudo apt-get install libboost-dev libboost-filesystem-dev`
  - Eigen3: `sudo apt-get install libeigen3-dev`
  - PCL: `sudo apt install libpcl-dev`
  - SuiteSparse: `sudo apt-get install  libsuitesparse-dev`
  - [magic_enum](https://github.com/Neargye/magic_enum):
    ```
    cd ~/your_path/magic_enum
    mkdir build && cd build
    sudo make install
    ```
  - [rapidcsv](https://github.com/d99kris/rapidcsv): alrady exist in the header, no need anything else

### Build:
  ```shell
  # download
  $ git clone https://github.com/GSO-soslab/msckf_dvio
  # ros dependencies
  $ rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
  # build
  $ catkin build msckf_dvio -DCMAKE_BUILD_TYPE=Release
  ```

## Usage: 
  ```shell
  ##### make sure change the your dataset path #####
  
  # run DVL-Pressure-Visual-IMU
  $ roslaunch msckf_dvio glrc_3_13_dpvio.launch
  # run DVL-Pressure-IMU
  $ roslaunch msckf_dvio glrc_3_13_dpio.launch
  # run EuRoC VIO (the very basic MSCKF VIO but with keyfram option)
  $ roslaunch msckf_dvio euroc_vio.launch

  ##### feel free to test your own dataset by configuring the yaml
  ##### you can set the system initialization parameters
  ```
## Dataset:
The underice dataset is available on request, please contact us for more information.
Our algorithm accepts following standard ROS message type:
  - IMU data: `sensor_msgs/Imu`  
  - DVL bottom track velocity: `geometry_msgs/TwistWithCovarianceStamped`
  - DVL pressure measurement: `sensor_msgs/FluidPressure`
  - DVL pointcloud: `sensor_msgs/PointCloud2`
  - Camera image: `sensor_msgs/Image` 



## Citation
If you find our code or paper useful, please cite

```bibtex
@inproceedings{MSCKF-DVIO_Zhao_IROS2023,
  author    = {Zhao, Lin and Zhou, Mingxi and Loose, Brice},
  title     = {Tightly-coupled Visual-{DVL}-Inertial Odometry for Robot-based Ice-water Boundary Exploration},
  booktitle = {2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2023},
  note      = {Accepted}
}
```

## Acknowledgement

This work is supported by the National Science Foundation (NSF) under the award [#1945924](https://www.nsf.gov/awardsearch/showAward?AWD_ID=1945924), and the Graduate School of Oceanography, University of Rhode Island. We also thank the field support from the Great Lake Research Center, Michigan Technological University

We adapted parts of code from [OpenVINS](https://github.com/rpng/open_vins), we want to thank them for releaseing the code for the public. If you think our code or paper is useful, plesae also cite the OpenVINS paper.